#include "gripper.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

// -------------------- Protocol constants --------------------
static constexpr uint8_t ID_BROADCAST    = 0xFE;
static constexpr uint8_t INST_PING       = 0x01;
static constexpr uint8_t INST_READ_DATA  = 0x02;
static constexpr uint8_t INST_WRITE_DATA = 0x03;
static constexpr uint8_t INST_REG_WRITE  = 0x04;
static constexpr uint8_t INST_ACTION     = 0x05;
static constexpr uint8_t INST_SYNC_WRITE = 0x83;

static inline void sleep_ms(int ms) { usleep(ms * 1000); }

static inline void push_u16_le(std::vector<uint8_t>& v, uint16_t x) {
  v.push_back(static_cast<uint8_t>(x & 0xFF));
  v.push_back(static_cast<uint8_t>((x >> 8) & 0xFF));
}

static inline int16_t read_i16_le(const uint8_t* p) {
  uint16_t u = static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
  return static_cast<int16_t>(u);
}

static inline void log_perror(const char* msg) { ::perror(msg); }

// ===================== SerialPort =====================
SerialPort::SerialPort() = default;
SerialPort::~SerialPort() { close(); }

bool SerialPort::open(const std::string& port, int baudrate, int timeout_ms) {
  timeout_ms_ = timeout_ms;

  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ < 0) {
    log_perror("open serial");
    return false;
  }

  if (!set_baudrate_termios_(baudrate)) {
    std::cerr << "Warning: failed to set baudrate=" << baudrate << "\n";
  }

  tcflush(fd_, TCIFLUSH);
  is_open_ = true;
  return true;
}

bool SerialPort::set_baudrate_termios_(int baudrate) {
  termios tio{};
  if (tcgetattr(fd_, &tio) != 0) {
    log_perror("tcgetattr");
    return false;
  }

  cfmakeraw(&tio);

  // 8N1
  tio.c_cflag &= ~PARENB;
  tio.c_cflag &= ~CSTOPB;
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;
  tio.c_cflag |= (CLOCAL | CREAD);

  // non-canonical read
  tio.c_cc[VMIN]  = 0;
  tio.c_cc[VTIME] = 0;

  speed_t sp;
  switch (baudrate) {
    case 9600: sp = B9600; break;
    case 57600: sp = B57600; break;
    case 115200: sp = B115200; break;
#ifdef B1000000
    case 1000000: sp = B1000000; break;
#endif
    default:
      std::cerr << "Unsupported baudrate in termios: " << baudrate
                << " (try 115200 or 1000000)\n";
      return false;
  }

  if (cfsetispeed(&tio, sp) != 0 || cfsetospeed(&tio, sp) != 0) {
    log_perror("cfsetispeed/cfsetospeed");
    return false;
  }

  if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
    log_perror("tcsetattr");
    return false;
  }

  return true;
}

void SerialPort::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  is_open_ = false;
}

bool SerialPort::isOpen() const { return is_open_; }

void SerialPort::resetInputBuffer() {
  if (!is_open_) return;
  tcflush(fd_, TCIFLUSH);
}

bool SerialPort::writeAll(const std::vector<uint8_t>& data) {
  if (!is_open_) return false;

  size_t total = 0;
  while (total < data.size()) {
    ssize_t n = ::write(fd_, data.data() + total, data.size() - total);
    if (n > 0) total += (size_t)n;
    else if (n < 0 && errno == EINTR) continue;
    else sleep_ms(1);
  }
  return true;
}

std::optional<std::vector<uint8_t>> SerialPort::readExact(size_t n, int timeout_ms_override) {
  if (!is_open_) return std::nullopt;

  int tmo = (timeout_ms_override >= 0) ? timeout_ms_override : timeout_ms_;
  std::vector<uint8_t> out(n);

  auto start = std::chrono::steady_clock::now();
  size_t got = 0;

  while (got < n) {
    int elapsed = (int)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count();
    int remain_ms = tmo - elapsed;
    if (remain_ms <= 0) return std::nullopt;

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd_, &rfds);

    timeval tv{};
    tv.tv_sec  = remain_ms / 1000;
    tv.tv_usec = (remain_ms % 1000) * 1000;

    int r = select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
    if (r <= 0) return std::nullopt;

    ssize_t rd = ::read(fd_, out.data() + got, n - got);
    if (rd > 0) got += (size_t)rd;
  }
  return out;
}

// ===================== BusServo =====================
BusServo::BusServo(const std::string& port, int baudrate, int timeout_ms, bool verbose)
    : verbose_(verbose) {
  if (!serial_.open(port, baudrate, timeout_ms)) {
    throw std::runtime_error("Failed to open serial port: " + port);
  }
  init_calibration_();
}

BusServo::~BusServo() { close(); }
void BusServo::close() { serial_.close(); }

void BusServo::log_hex_(const std::string& dir, const std::vector<uint8_t>& data) {
  if (!verbose_ || data.empty()) return;
  std::cout << "[" << dir << "] ";
  for (auto b : data) {
    std::cout << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
              << (int)b << " ";
  }
  std::cout << std::dec << "\n";
}

uint8_t BusServo::calc_checksum_(uint8_t id, uint8_t length, uint8_t instruction,
                                 const std::vector<uint8_t>& params) {
  uint32_t total = id + length + instruction;
  for (auto p : params) total += p;
  return (uint8_t)((~total) & 0xFF);
}

std::vector<uint8_t> BusServo::send_packet_(uint8_t id, uint8_t instruction,
                                            const std::vector<uint8_t>& params) {
  uint8_t length = (uint8_t)(params.size() + 2);
  uint8_t checksum = calc_checksum_(id, length, instruction, params);

  std::vector<uint8_t> pkt;
  pkt.reserve(2 + 1 + 1 + 1 + params.size() + 1);
  pkt.push_back(0xFF);
  pkt.push_back(0xFF);
  pkt.push_back(id);
  pkt.push_back(length);
  pkt.push_back(instruction);
  pkt.insert(pkt.end(), params.begin(), params.end());
  pkt.push_back(checksum);

  //log_hex_("TX", pkt);
  serial_.resetInputBuffer();
  serial_.writeAll(pkt);
  return pkt;
}

BusServo::Resp BusServo::receive_packet_() {
  Resp r;
  std::vector<uint8_t> raw;

  auto hdr = serial_.readExact(2);
  if (!hdr) { r.msg = "Timeout (No Header)"; return r; }
  raw.insert(raw.end(), hdr->begin(), hdr->end());

  if ((*hdr)[0] != 0xFF || (*hdr)[1] != 0xFF) {
    log_hex_("RX", raw);
    r.msg = "Header Error";
    return r;
  }

  auto idb = serial_.readExact(1);
  if (!idb) { r.msg = "Timeout (No ID)"; return r; }
  uint8_t resp_id = (*idb)[0];

  auto lenb = serial_.readExact(1);
  if (!lenb) { r.msg = "Timeout (No Length)"; return r; }
  uint8_t length = (*lenb)[0];

  auto remain = serial_.readExact(length);
  if (!remain) { r.msg = "Timeout/Incompleted Packet"; return r; }

  uint8_t error = (*remain)[0];
  uint8_t recv_checksum = (*remain)[length - 1];

  std::vector<uint8_t> params;
  if (length >= 2) {
    params.insert(params.end(), remain->begin() + 1, remain->end() - 1);
  }

  uint8_t calc = calc_checksum_(resp_id, length, error, params);
  if (calc != recv_checksum) {
    r.msg = "Checksum Error";
    return r;
  }

  r.ok = true;
  r.error = error;
  r.params = std::move(params);
  return r;
}

uint8_t BusServo::ping(uint8_t servo_id) {
  send_packet_(servo_id, INST_PING, {});
  auto resp = receive_packet_();
  if (!resp.ok) return 0xFF;
  return resp.error;
}

std::optional<std::vector<uint8_t>> BusServo::read_data(uint8_t servo_id, uint8_t address, uint8_t length) {
  send_packet_(servo_id, INST_READ_DATA, {address, length});
  auto resp = receive_packet_();
  if (!resp.ok) return std::nullopt;
  if (resp.params.size() == length) return resp.params;
  return std::nullopt;
}

std::pair<std::vector<uint8_t>, uint8_t>
BusServo::write_data(uint8_t servo_id, uint8_t address, const std::vector<uint8_t>& values) {
  std::vector<uint8_t> params;
  params.reserve(1 + values.size());
  params.push_back(address);
  params.insert(params.end(), values.begin(), values.end());

  send_packet_(servo_id, INST_WRITE_DATA, params);
  if (servo_id != ID_BROADCAST) {
    auto resp = receive_packet_();
    if (!resp.ok) return {{}, 0xFF};
    return {resp.params, resp.error};
  }
  return {{}, 0};
}

std::pair<std::vector<uint8_t>, uint8_t>
BusServo::reg_write(uint8_t servo_id, uint8_t address, const std::vector<uint8_t>& values) {
  std::vector<uint8_t> params;
  params.reserve(1 + values.size());
  params.push_back(address);
  params.insert(params.end(), values.begin(), values.end());

  send_packet_(servo_id, INST_REG_WRITE, params);
  if (servo_id != ID_BROADCAST) {
    auto resp = receive_packet_();
    if (!resp.ok) return {{}, 0xFF};
    return {resp.params, resp.error};
  }
  return {{}, 0};
}

void BusServo::action(uint8_t servo_id) {
  send_packet_(servo_id, INST_ACTION, {});
}

void BusServo::sync_write(uint8_t address, uint8_t data_len, const std::vector<std::vector<uint8_t>>& servo_data) {
  std::vector<uint8_t> params;
  params.push_back(address);
  params.push_back(data_len);
  for (const auto& item : servo_data) {
    params.insert(params.end(), item.begin(), item.end());
  }
  send_packet_(ID_BROADCAST, INST_SYNC_WRITE, params);
}

void BusServo::move_servo(uint8_t servo_id, uint16_t position, uint16_t speed) {
  const uint16_t pwm = 0x1000;

  std::vector<uint8_t> item;
  item.reserve(1 + 6);
  item.push_back(servo_id);
  push_u16_le(item, position);
  push_u16_le(item, pwm);
  push_u16_le(item, speed);

  // 0x2A: 你原代码用的地址
  sync_write(0x2A, 6, {item});
}

std::optional<BusServo::SensorData> BusServo::read_sensor_data(uint8_t servo_id) {
  auto data = read_data(servo_id, 0x38, 8);
  if (!data || data->size() != 8) return std::nullopt;

  SensorData s;
  s.id = servo_id;
  s.position = read_i16_le(&(*data)[0]);
  s.speed    = read_i16_le(&(*data)[2]);
  s.load     = read_i16_le(&(*data)[4]);
  s.voltage  = (*data)[6];
  s.temperature = (*data)[7];
  return s;
}

int BusServo::interpolate_(double value, const std::vector<std::pair<double, int>>& table) {
  if (table.empty()) return 0;
  if (value <= table.front().first) return table.front().second;
  if (value >= table.back().first)  return table.back().second;

  for (size_t i = 0; i + 1 < table.size(); ++i) {
    double x0 = table[i].first;
    double x1 = table[i + 1].first;
    int y0 = table[i].second;
    int y1 = table[i + 1].second;

    if (x0 <= value && value <= x1) {
      double ratio = (value - x0) / (x1 - x0);
      double y = y0 + ratio * (double)(y1 - y0);
      return (int)std::lround(y);
    }
  }
  return table.back().second;
}

// 反向插值：给定 servo_pos(0~4096) -> mm
double BusServo::inverse_interpolate_mm_(int servo_pos,
                                        const std::vector<std::pair<double, int>>& table) {
  if (table.empty()) return 0.0;

  // table: (mm, pos)，注意 pos 可能是递减的（你 100mm 表就是递减）
  // 我们遍历相邻段，找到 servo_pos 落在哪一段之间
  for (size_t i = 0; i + 1 < table.size(); ++i) {
    double mm0 = table[i].first;
    int    p0  = table[i].second;
    double mm1 = table[i + 1].first;
    int    p1  = table[i + 1].second;

    // 判断 servo_pos 是否在 [p0,p1] 或 [p1,p0] 之间
    int lo = std::min(p0, p1);
    int hi = std::max(p0, p1);
    if (servo_pos >= lo && servo_pos <= hi) {
      // 线性插值：用 position 在 p0->p1 的比例，映射到 mm0->mm1
      double denom = double(p1 - p0);
      if (std::abs(denom) < 1e-9) return mm0;
      double t = (double(servo_pos) - double(p0)) / denom;  // t in [0,1] (可能反向)
      double mm = mm0 + t * (mm1 - mm0);
      return mm;
    }
  }

  // 超出表范围：夹到最近端
  // 如果位置更接近 front / back，就返回对应 mm
  int pf = table.front().second;
  int pb = table.back().second;
  if (std::abs(servo_pos - pf) < std::abs(servo_pos - pb)) return table.front().first;
  return table.back().first;
}

// 读取舵机当前开口（mm）
double BusServo::get_position_mm(uint8_t servo_id,
                                                const std::string& gripper_type) {
  auto it = gripper_calib_.find(gripper_type);
  if (it == gripper_calib_.end()) return 0;

  auto s = read_sensor_data(servo_id);
  if (!s) return 0;

  int pos = s->position; // 原始回读
  // 有些舵机可能返回负数或超过范围，你可 clamp 一下
  pos = std::clamp(pos, 0, 4096);

  double mm = inverse_interpolate_mm_(pos, it->second);
  return mm;
}


void BusServo::init_calibration_() {
  gripper_calib_["100mm"] = {
    {0, 3434}, {10, 3060}, {20, 2824}, {30, 2646}, {40, 2490},
    {50, 2354}, {60, 2226}, {70, 2095}, {80, 1965}, {90, 1810}, {100, 1594}
  };

  gripper_calib_["50mm"] = {
    {0, 2048}, {5, 1900}, {10, 1775}, {15, 1645}, {20, 1530},
    {25, 1430}, {30, 1320}, {35, 1215}, {40, 1085}, {45, 960}, {50, 798}
  };
}

void BusServo::set_gripper_position(uint8_t servo_id,
                                    const std::string& gripper_type,
                                    double position_mm,
                                    uint16_t speed) {
  auto it = gripper_calib_.find(gripper_type);
  if (it == gripper_calib_.end()) {
    std::cerr << "Error: Unknown gripper type '" << gripper_type << "'\n";
    return;
  }

  int target = interpolate_(position_mm, it->second);
  target = std::clamp(target, 0, 4096);

  if (verbose_) {
    std::cout << "--> Gripper: type=" << gripper_type
              << " target_mm=" << position_mm
              << " servo_pos=" << target << "\n";
  }

  move_servo(servo_id, (uint16_t)target, speed);
}

void BusServo::set_gripper_openclose(uint8_t servo_id,
                                     const std::string& gripper_type,
                                     int cmd01,
                                     uint16_t speed) {
  auto it = gripper_calib_.find(gripper_type);
  if (it == gripper_calib_.end()) {
    std::cerr << "Error: Unknown gripper type '" << gripper_type << "'\n";
    return;
  }

  cmd01 = (cmd01 != 0) ? 1 : 0;
  const auto& table = it->second;

  double close_mm = table.front().first;
  double open_mm  = table.back().first;

  double target_mm = cmd01 ? open_mm : close_mm;
  set_gripper_position(servo_id, gripper_type, target_mm, speed);
}

uint8_t BusServo::get_position(uint8_t servo_id)
 {
  return 0;
}


// ===================== main (test) =====================
// int main(int argc, char** argv) {
//   (void)argc; (void)argv; // 不用参数就这样消 warning

//   try {
//     BusServo servo("/dev/ttyUSB0", 1000000, 150, true);

//     uint8_t servo_id = 10; // 你写死 10 就行

//     std::cout << "--- TEST: 0(close) / 1(open) ---\n";
//     servo.set_gripper_openclose(servo_id, "100mm", 0, 800);
//     sleep_ms(5000);
//     servo.set_gripper_openclose(servo_id, "100mm", 1, 800);
//     sleep_ms(2000);

//     servo.close();
//   } catch (const std::exception& e) {
//     std::cerr << "Error: " << e.what() << "\n";
//     return 1;
//   }
//   return 0;
// }
