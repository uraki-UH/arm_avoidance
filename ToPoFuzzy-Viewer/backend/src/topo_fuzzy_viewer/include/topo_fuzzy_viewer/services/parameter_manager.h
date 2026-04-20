#pragma once

#include <vector>
#include <string>
#include <optional>
#include <variant>

namespace core {

/**
 * @brief パラメータ型の列挙
 */
enum class ParamType { BOOL, INT, FLOAT, STRING };

/**
 * @brief パラメータのメタ情報と現在値
 */
struct ParameterInfo {
    std::string name;
    std::string description;
    ParamType type;
    
    // 数値型の場合の範囲制約
    double min = 0.0;
    double max = 100.0;
    double step = 1.0;
    
    // 現在値
    std::variant<bool, int64_t, double, std::string> value;
    
    ParameterInfo() = default;
    ParameterInfo(const std::string& n, const std::string& desc, ParamType t,
                  double min_, double max_, double step_, double val)
        : name(n), description(desc), type(t), min(min_), max(max_), step(step_), value(val) {}

    ParameterInfo(const std::string& n, const std::string& desc, ParamType t,
                  double min_, double max_, double step_, const std::string& val)
        : name(n), description(desc), type(t), min(min_), max(max_), step(step_), value(val) {}
};

/**
 * @brief ノードのパラメータ一覧
 */
struct NodeParameters {
    std::string node_name;
    std::vector<ParameterInfo> parameters;
};

/**
 * @brief 抽象パラメータマネージャー
 * 
 * ROS2, gRPC等、様々なパラメータプロバイダを
 * 統一的に扱うためのインターフェース
 */
class ParameterManager {
public:
    virtual ~ParameterManager() = default;

    /**
     * @brief 対象ノードのパラメータ定義を取得
     * @return NodeParameters (パラメータ一覧)
     */
    virtual std::optional<NodeParameters> getNodeParameters() = 0;

    /**
     * @brief パラメータを設定
     * @param param_name パラメータ名
     * @param value 新しい値
     * @return 成功したら true
     */
    virtual bool setParameter(const std::string& param_name, double value) = 0;
    virtual bool setParameter(const std::string& param_name, const std::string& value) { return false; }
};

/**
 * @brief スタンドアロンモード用のダミー実装
 */
class StandaloneParameterManager : public ParameterManager {
public:
    std::optional<NodeParameters> getNodeParameters() override {
        return std::nullopt; // スタンドアロンではパラメータ操作なし
    }

    bool setParameter(const std::string& /*param_name*/, double /*value*/) override {
        return false;
    }
};

} // namespace core
