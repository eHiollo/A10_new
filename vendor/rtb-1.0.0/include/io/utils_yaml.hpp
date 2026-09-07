#ifndef RTB_IO_YAML_HELPER_HPP_
#define RTB_IO_YAML_HELPER_HPP_

#ifdef RTB_YAML_ENABLED

#include <fstream>
#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>

namespace rtb {
namespace io {

class YamlHelper {
public:
    // =========================================================
    // 文件操作 (macOS/Ubuntu 通用)
    // =========================================================

    /**
     * @brief 安全加载 YAML 文件
     */
    static auto loadFile(const std::string& filename) -> YAML::Node {
        try {
            // [兼容性] .c_str() 对旧版 Ubuntu 的 ifstream 和 yaml-cpp 0.5 更友好
            return YAML::LoadFile(filename.c_str());
        } catch (...) {
            // [兼容性] macOS(libc++) 和 Ubuntu(libstdc++) 异常符号不同，统一捕获
            std::cerr << "[YamlHelper] Error loading " << filename << std::endl;
            return YAML::Node();  // 返回 Null Node
        }
    }

    /**
     * @brief 加载文件用于更新 (增量模式)
     */
    static auto loadOrCreate(const std::string& filename) -> YAML::Node {
        std::ifstream fin(filename.c_str());  // [兼容性] c_str()
        if (fin.good()) {
            try {
                YAML::Node node = YAML::Load(fin);
                // 确保是 Map 类型，否则后续操作会崩
                if (node.IsNull() || !node.IsMap()) {
                    return YAML::Node(YAML::NodeType::Map);
                }
                return node;
            } catch (...) {
                std::cerr << "[YamlHelper] File corrupted, creating new: " << filename << std::endl;
                return YAML::Node(YAML::NodeType::Map);
            }
        }
        return YAML::Node(YAML::NodeType::Map);
    }

    /**
     * @brief 保存文件
     */
    static auto saveFile(const std::string& filename, const YAML::Node& node) -> bool {
        try {
            std::ofstream fout(filename.c_str());
            if (!fout.is_open())
                return false;
            fout << node;
            fout.close();
            return true;
        } catch (...) {
            std::cerr << "[YamlHelper] Failed to save: " << filename << std::endl;
            return false;
        }
    }

    // =========================================================
    // 读写操作 (核心：使用下标循环，避开迭代器差异)
    // =========================================================

    template <typename T>
    static auto readArray(const YAML::Node& node, const std::string& key, T* ptr, size_t size) -> bool {
        if (!node[key])
            return false;

        // [兼容性] 0.5(Ubuntu) 和 0.8(macOS) 对 operator[] const 的处理不同
        // 建议拷贝 Node 或使用非 const 引用，这里使用 const Node& 配合下标在两个版本都安全
        const YAML::Node& list = node[key];

        if (!list.IsSequence() || list.size() != size) {
            // std::cerr << "[YamlHelper] Size mismatch: " << key << std::endl;
            return false;
        }

        try {
            // [兼容性] 绝对不要用 for(auto it : list)，macOS Clang 和 GCC 5.4 行为不一致
            for (size_t i = 0; i < size; ++i) {
                // [兼容性] macOS(0.8+) 必须显式 .as<T>()，不允许隐式转换
                ptr[i] = list[i].as<T>();
            }
            return true;
        } catch (...) {
            return false;
        }
    }

    template <typename T>
    static auto writeArray(YAML::Node& node, const std::string& key, const T* ptr, size_t size) -> void {
        YAML::Node list_node;
        for (size_t i = 0; i < size; ++i) {
            list_node.push_back(ptr[i]);
        }
        node[key] = list_node;
    }

    // =========================================================
    // 矩阵操作 (4x4 等)
    // =========================================================

    template <typename T>
    static auto readMatrix(const YAML::Node& node, const std::string& key, T* out_data, int rows, int cols) -> bool {
        if (!node[key])
            return false;
        const YAML::Node& matrix_node = node[key];

        if (!matrix_node.IsSequence() || static_cast<int>(matrix_node.size()) != rows)
            return false;

        try {
            for (int r = 0; r < rows; ++r) {
                const YAML::Node& row_node = matrix_node[r];
                if (!row_node.IsSequence() || static_cast<int>(row_node.size()) != cols)
                    return false;

                for (int c = 0; c < cols; ++c) {
                    out_data[r * cols + c] = row_node[c].as<T>();
                }
            }
            return true;
        } catch (...) {
            return false;
        }
    }

    template <typename T>
    static auto writeMatrix(YAML::Node& node, const std::string& key, const T* data, int rows, int cols) -> void {
        YAML::Node matrix_node;
        for (int r = 0; r < rows; ++r) {
            YAML::Node row_node;
            for (int c = 0; c < cols; ++c) {
                row_node.push_back(data[r * cols + c]);
            }
            matrix_node.push_back(row_node);
        }
        node[key] = matrix_node;
    }

    // =========================================================
    // 基础类型与原子更新
    // =========================================================

    template <typename T>
    static auto read(const YAML::Node& node, const std::string& key, T& out_val) -> bool {
        if (!node[key])
            return false;
        try {
            out_val = node[key].as<T>();
            return true;
        } catch (...) {
            return false;
        }
    }

    template <typename T>
    static auto write(YAML::Node& node, const std::string& key, const T& value) -> void {
        node[key] = value;
    }

    template <typename T>
    static auto updateFileArray(const std::string& filename, const std::string& key, const T* ptr, size_t size) -> bool {
        YAML::Node node = loadOrCreate(filename);
        writeArray(node, key, ptr, size);
        return saveFile(filename, node);
    }

    template <typename T>
    static auto updateFileMatrix(
        const std::string& filename, const std::string& key, const T* data, int rows, int cols) -> bool {
        YAML::Node node = loadOrCreate(filename);
        writeMatrix(node, key, data, rows, cols);
        return saveFile(filename, node);
    }
};

}  // namespace io

}  // namespace rtb

#endif  // RTB_YAML_ENABLED

#endif  // RTB_IO_YAML_HELPER_HPP_
