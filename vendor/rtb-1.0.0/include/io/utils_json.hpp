#ifndef RTB_IO_JSON_HELPER_HPP_
#define RTB_IO_JSON_HELPER_HPP_

#include <fstream>
#include <iomanip>  // for std::setw
#include <iostream>
#include <string>
#include <vector>

// 请确保 json.hpp 在你的 include 路径中
#include "io/json.hpp"

namespace rtb {
namespace io {

using json = nlohmann::json;

class JsonHelper {
public:
    // =========================================================
    // 文件操作 (Load / Save)
    // =========================================================

    /**
     * @brief 加载 JSON 文件
     * @return 成功返回 json 对象，失败返回空对象
     */
    static auto loadFile(const std::string& filename) -> json {
        std::ifstream fin(filename);
        if (fin.good()) {
            try {
                json j;
                fin >> j;
                return j;
            } catch (const json::parse_error& e) {
                std::cerr << "[JsonHelper] Parse error in " << filename << ": " << e.what() << std::endl;
                return json::object();
            }
        }
        return json::object();  // 文件不存在返回空对象 {}
    }

    /**
     * @brief 加载文件用于更新 (如果不存在则创建)
     */
    static auto loadOrCreate(const std::string& filename) -> json {
        json j = loadFile(filename);
        if (j.is_null()) {
            return json::object();
        }
        return j;
    }

    /**
     * @brief 保存 JSON 文件 (自动美化缩进)
     */
    static auto saveFile(const std::string& filename, const json& j) -> bool {
        std::ofstream fout(filename);
        if (!fout.is_open())
            return false;
        // setw(4) 用于美化输出 (pretty print)，否则是压缩的一行
        fout << std::setw(4) << j << std::endl;
        return true;
    }

    // =========================================================
    // 读写操作 (STL 风格，极其简洁)
    // =========================================================

    /**
     * @brief 读取数组到 C 数组
     */
    template <typename T>
    static auto readArray(const json& j, const std::string& key, T* ptr, size_t size) -> bool {
        if (!j.contains(key) || !j[key].is_array())
            return false;

        const auto& arr = j[key];
        if (arr.size() != size)
            return false;

        for (size_t i = 0; i < size; ++i) {
            ptr[i] = arr[i].get<T>();
        }
        return true;
    }

    /**
     * @brief 写入 C 数组为 JSON 数组
     */
    template <typename T>
    static auto writeArray(json& j, const std::string& key, const T* ptr, size_t size) -> void {
        // nlohmann::json 支持直接用 vector 初始化
        std::vector<T> vec(ptr, ptr + size);
        j[key] = vec;
    }

    /**
     * @brief 读取 4x4 矩阵
     */
    template <typename T>
    static auto readMatrix(const json& j, const std::string& key, T* out_data, int rows, int cols) -> bool {
        if (!j.contains(key) || !j[key].is_array())
            return false;

        const auto& matrix = j[key];
        if (static_cast<int>(matrix.size()) != rows)
            return false;

        for (int r = 0; r < rows; ++r) {
            const auto& row = matrix[r];
            if (!row.is_array() || static_cast<int>(row.size()) != cols)
                return false;

            for (int c = 0; c < cols; ++c) {
                out_data[r * cols + c] = row[c].get<T>();
            }
        }
        return true;
    }

    /**
     * @brief 写入 4x4 矩阵
     */
    template <typename T>
    static auto writeMatrix(json& j, const std::string& key, const T* data, int rows, int cols) -> void {
        std::vector<std::vector<T>> matrix;
        for (int r = 0; r < rows; ++r) {
            std::vector<T> row;
            for (int c = 0; c < cols; ++c) {
                row.push_back(data[r * cols + c]);
            }
            matrix.push_back(row);
        }
        j[key] = matrix;
    }

    // =========================================================
    // 基础读写
    // =========================================================

    template <typename T>
    static auto read(const json& j, const std::string& key, T& out_val) -> bool {
        if (!j.contains(key))
            return false;
        out_val = j[key].get<T>();
        return true;
    }

    template <typename T>
    static auto write(json& j, const std::string& key, const T& value) -> void { j[key] = value; }

    // =========================================================
    // 原子更新 (One-Liner)
    // =========================================================

    template <typename T>
    static auto updateFileArray(const std::string& filename, const std::string& key, const T* ptr, size_t size) -> bool {
        json j = loadOrCreate(filename);
        writeArray(j, key, ptr, size);
        return saveFile(filename, j);
    }
};

}  // namespace io
}  // namespace rtb

#endif  // RTB_IO_JSON_HELPER_HPP_