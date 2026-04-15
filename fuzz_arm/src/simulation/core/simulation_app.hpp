/*
 * simulation_app.hpp
 *
 * SimulationApp クラスの定義
 * シミュレーションアプリケーションのメインロジックをカプセル化します。
 * SimulationState の管理、初期化、メインループ、描画処理を担当します。
 */

#pragma once

#include <fstream>
#include <string>

// 必要なヘッダーのインクルード
// (既存のSimulationStateで使われているものを適宜インクルード)
#include "simulation/core/simulation_state.hpp"
#include <Eigen/Dense>
#include <drawstuff/drawstuff.h>
#include <map>
#include <memory>
#include <ode/ode.h>
#include <vector>

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawLine dsDrawLineD
#endif

typedef dReal dReal4[4];

namespace robot_sim {
namespace simulation {

/**
 * @brief シミュレーションアプリケーションクラス
 *
 * シングルトンパターンを採用し、CスタイルのDrawStuffコールバックから
 * メンバ関数にアクセスできるようにしています。
 */
class SimulationApp {
public:
  /**
   * @brief シングルトンインスタンスへのアクセス
   * @return SimulationApp& インスタンスへの参照
   */
  static SimulationApp &instance();

  /**
   * @brief アプリケーションの実行
   * @param argc コマンドライン引数の数
   * @param argv コマンドライン引数
   */
  void run(int argc, char **argv);

  // --- DrawStuff用 静的コールバック関数 ---
  static void startCallback();
  static void stepCallback(int pause);
  static void commandCallback(int cmd);
  static void stopCallback();

private:
  // コンストラクタ・デストラクタ（シングルトン化のためprivate）
  SimulationApp();
  ~SimulationApp();

  // コピー禁止
  SimulationApp(const SimulationApp &) = delete;
  SimulationApp &operator=(const SimulationApp &) = delete;

  // --- 内部ロジック ---

  /**
   * @brief シミュレーションの初期化
   * @param config_file 設定ファイルパス
   * @return 成功したらtrue
   */
  bool initialize(const std::string &config_file);

  /**
   * @brief シミュレーションの1ステップ更新
   * @param pause 一時停止中かどうか（1なら一時停止）
   */
  void update(int pause);

  /**
   * @brief 描画処理
   */
  void draw();

  /**
   * @brief 終了処理
   */
  void finalize();

  /**
   * @brief カメラ設定の更新
   */
  void updateCamera();

  /**
   * @brief GUIの描画 (Dear ImGui)
   */
  void renderGui();

  /**
   * @brief ユーザー入力コマンドの処理
   * @param cmd コマンドキー
   */
  void handleCommand(int cmd);

  /**
   * @brief マニピュラリティの可視化
   */
  void drawManipulabilityViz();

  /**
   * @brief ジオメトリの描画ヘルパー
   * @param g ジオメトリID
   * @param color 色
   * @param use_texture テクスチャを使用するかどうか
   */
  void drawGeom(dGeomID g, const dReal4 &color, bool use_texture = false);

  // --- ヘルパー関数 ---
  void handlePickRandomPosture();
  void handlePickRandomGngNode();
  void planAndExecutePath(const Eigen::VectorXf &start,
                          const Eigen::VectorXf &goal);
  void startExperiment(const std::string &method = "");
  void resetExperiment();

  /**
   * @brief Generate simplified visuals for all meshes in the robot model
   */
  void generateSimplifiedRobotVisuals();
  void syncKinematics();

  // --- メンバ変数 ---

  // シミュレーション状態データ
  SimulationState state_;

  // ログ用CSV
  std::ofstream manipulability_csv_;
  std::ofstream reachability_csv_; // Added for experimental logging
  std::ofstream robustness_csv_;   // Added for perturbation sweep logging

  // タイマー関連
  double safety_update_time_ms_ = 0.0;
  double env_update_time_ms_ = 0.0;
  double last_safety_time_ = 0.0;
  double last_env_time_ = 0.0; // 追加

  // DrawStuff用の定義
  static dsFunctions fn_;
};

} // namespace simulation
} // namespace robot_sim
