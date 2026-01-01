#include <rclcpp/rclcpp.hpp>
#include <sqlite3.h>

#include <string>
#include <filesystem>
#include <mutex>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <cmath>


#include "wifi_interface/msg/wifi_fused.hpp"
#include "wifi_interface/srv/get_heatmap.hpp"          // GetHeatmap.srv
#include "std_srvs/srv/trigger.hpp"                    // start/stop에 사용
#include "wifi_interface/srv/list_sessions.hpp"
#include "wifi_interface/srv/delete_session.hpp"
#include "wifi_interface/srv/list_ssids.hpp"


class WifiDbNode : public rclcpp::Node
{
public:
  WifiDbNode() : Node("wifi_db_node")
  {
    device_info_ = "turtlebot3";
    db_path_ = "/home/ros/intel-edge-ai-sw-8/2601_4th_proj_dahyeon/"
  "trunk/data/wifi/wifi_heatmap.db";

    std::filesystem::create_directories(std::filesystem::path(db_path_).parent_path());

    if (sqlite3_open(db_path_.c_str(), &db_) != SQLITE_OK) {
      RCLCPP_FATAL(get_logger(), "Cannot open DB: %s", sqlite3_errmsg(db_));
      rclcpp::shutdown();
      return;
    }

    // 성능/안정 옵션(선택)
    sqlite3_exec(db_, "PRAGMA journal_mode=WAL;", nullptr, nullptr, nullptr);
    sqlite3_exec(db_, "PRAGMA synchronous=NORMAL;", nullptr, nullptr, nullptr);

    createTables();
    createIndexes();

    // ===== subscriber: /wifi/fused =====
    sub_ = create_subscription<wifi_interface::msg::WifiFused>(
      "/wifi/fused", rclcpp::QoS(10),
      std::bind(&WifiDbNode::onFused, this, std::placeholders::_1)
    );

    // ===== service: /db/start_session (Trigger) =====
    srv_start_ = create_service<std_srvs::srv::Trigger>(
      "/db/start_session",
      std::bind(&WifiDbNode::onStartSession, this, std::placeholders::_1, std::placeholders::_2)
    );

    // ===== service: /db/stop_session (Trigger) =====
    srv_stop_ = create_service<std_srvs::srv::Trigger>(
      "/db/stop_session",
      std::bind(&WifiDbNode::onStopSession, this, std::placeholders::_1, std::placeholders::_2)
    );

    // ===== service: /db/get_heatmap (GetHeatmap) =====
    srv_heatmap_ = create_service<wifi_interface::srv::GetHeatmap>(
      "/db/get_heatmap",
      std::bind(&WifiDbNode::onGetHeatmap, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(get_logger(), "wifi_db_node started. DB=%s", db_path_.c_str());
    RCLCPP_INFO(get_logger(), "Services: /db/start_session, /db/stop_session, /db/get_heatmap");

    srv_list_sessions_ = create_service<wifi_interface::srv::ListSessions>(
      "/db/list_sessions",
      std::bind(&WifiDbNode::onListSessions, this,
              std::placeholders::_1, std::placeholders::_2)
      );

  // ===== service: /db/delete_session (DeleteSession) =====
    srv_delete_ = create_service<wifi_interface::srv::DeleteSession>(
      "/db/delete_session",
      std::bind(&WifiDbNode::onDeleteSession, this, std::placeholders::_1, std::placeholders::_2)
    );

  // ===== service: /db/list_ssids (ListSsids) =====
  srv_list_ssids_ = create_service<wifi_interface::srv::ListSsids>(
    "/db/list_ssids",
    std::bind(&WifiDbNode::onListSsids, this, std::placeholders::_1, std::placeholders::_2)
  );

RCLCPP_INFO(get_logger(), "Services +: /db/delete_session, /db/list_ssids");

}

  ~WifiDbNode()
  {
    if (db_) {
      sqlite3_close(db_);
      db_ = nullptr;
    }
  }

private:
  // ===== schema =====
  void createTables()
  {
    const char *sql_wifi =
      "CREATE TABLE IF NOT EXISTS wifi_data ("
      "id INTEGER PRIMARY KEY AUTOINCREMENT,"
      "session_id TEXT NOT NULL,"
      "device_info TEXT NOT NULL,"
      "ssid TEXT NOT NULL,"
      "grid_id_x REAL NOT NULL,"
      "grid_id_y REAL NOT NULL,"
      "rssi_value REAL NOT NULL,"
      "timestamp TEXT DEFAULT (strftime('%Y-%m-%dT%H:%M:%SZ', 'now'))"
      ");";

    const char *sql_sessions =
      "CREATE TABLE IF NOT EXISTS sessions ("
      "session_id TEXT PRIMARY KEY,"
      "device_info TEXT NOT NULL,"
      "started_at TEXT DEFAULT (strftime('%Y-%m-%dT%H:%M:%SZ', 'now')),"
      "ended_at TEXT"
      ");";

    char *err = nullptr;
    if (sqlite3_exec(db_, sql_wifi, nullptr, nullptr, &err) != SQLITE_OK) {
      RCLCPP_ERROR(get_logger(), "Create wifi_data error: %s", err);
      sqlite3_free(err);
    }
    err = nullptr;
    if (sqlite3_exec(db_, sql_sessions, nullptr, nullptr, &err) != SQLITE_OK) {
      RCLCPP_ERROR(get_logger(), "Create sessions error: %s", err);
      sqlite3_free(err);
    }
  }

  void createIndexes()
  {
    const char *sql =
      "CREATE INDEX IF NOT EXISTS idx_wifi_session ON wifi_data(session_id);"
      "CREATE INDEX IF NOT EXISTS idx_wifi_ssid ON wifi_data(ssid);"
      "CREATE INDEX IF NOT EXISTS idx_wifi_rssi ON wifi_data(rssi_value);"
      "CREATE INDEX IF NOT EXISTS idx_wifi_time ON wifi_data(timestamp);";

    char *err = nullptr;
    if (sqlite3_exec(db_, sql, nullptr, nullptr, &err) != SQLITE_OK) {
      RCLCPP_WARN(get_logger(), "Create index warn: %s", err);
      sqlite3_free(err);
    }
  }

  // ===== utils =====
  static std::string makeSessionIdUtc()
  {
    // 예: 20260101_093012Z 형태
    using namespace std::chrono;
    auto now = system_clock::now();
    std::time_t t = system_clock::to_time_t(now);

    std::tm tm{};
    gmtime_r(&t, &tm);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%SZ");
    return oss.str();
  }

  bool hasActiveSessionLocked() const
  {
    return !current_session_id_.empty();
  }

  // ===== subscriber =====
  void onFused(const wifi_interface::msg::WifiFused::SharedPtr msg)
  {
    if (!msg) return;
    if (msg->ssid.empty()) return;     // 의미 없는 데이터 차단

    std::lock_guard<std::mutex> lk(db_mtx_);

    // 세션이 시작되지 않았다면 저장 안 함(정책)
    if (!hasActiveSessionLocked()) return;

    const char *sql =
      "INSERT INTO wifi_data "
      "(session_id, device_info, ssid, grid_id_x, grid_id_y, rssi_value) "
      "VALUES (?, ?, ?, ?, ?, ?);";

    sqlite3_stmt *stmt = nullptr;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
      RCLCPP_ERROR(get_logger(), "Prepare failed: %s", sqlite3_errmsg(db_));
      return;
    }

    sqlite3_bind_text(stmt, 1, current_session_id_.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 2, device_info_.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 3, msg->ssid.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_double(stmt, 4, msg->x);
    sqlite3_bind_double(stmt, 5, msg->y);
    sqlite3_bind_double(stmt, 6, msg->rssi);

    if (sqlite3_step(stmt) != SQLITE_DONE) {
      RCLCPP_ERROR(get_logger(), "Insert failed: %s", sqlite3_errmsg(db_));
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "DB insert: sid=%s ssid=%s x=%.2f y=%.2f rssi=%d",
        current_session_id_.c_str(), msg->ssid.c_str(), msg->x, msg->y, msg->rssi
      );
    }

    sqlite3_finalize(stmt);
  }

  // ===== service: start_session =====
  void onStartSession(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    std::lock_guard<std::mutex> lk(db_mtx_);

    if (hasActiveSessionLocked()) {
      res->success = true;
      res->message = current_session_id_; // 이미 진행 중이면 현재 세션 id 반환
      return;
    }

    const std::string sid = makeSessionIdUtc();

    // sessions 테이블에 기록
    const char *sql = "INSERT INTO sessions(session_id, device_info) VALUES(?, ?);";
    sqlite3_stmt *stmt = nullptr;

    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
      res->success = false;
      res->message = std::string("Prepare failed: ") + sqlite3_errmsg(db_);
      return;
    }

    sqlite3_bind_text(stmt, 1, sid.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 2, device_info_.c_str(), -1, SQLITE_TRANSIENT);

    if (sqlite3_step(stmt) != SQLITE_DONE) {
      res->success = false;
      res->message = std::string("Insert session failed: ") + sqlite3_errmsg(db_);
      sqlite3_finalize(stmt);
      return;
    }

    sqlite3_finalize(stmt);

    current_session_id_ = sid;

    res->success = true;
    res->message = sid; // Trigger message에 session_id를 넣어 Qt가 사용
    RCLCPP_INFO(get_logger(), "Session START: %s", sid.c_str());
  }

  // ===== service: stop_session =====
  void onStopSession(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    std::lock_guard<std::mutex> lk(db_mtx_);

    if (!hasActiveSessionLocked()) {
      res->success = true;
      res->message = ""; // 세션 없음
      return;
    }

    const std::string sid = current_session_id_;

    const char *sql =
      "UPDATE sessions SET ended_at = strftime('%Y-%m-%dT%H:%M:%SZ', 'now') "
      "WHERE session_id = ?;";

    sqlite3_stmt *stmt = nullptr;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
      res->success = false;
      res->message = std::string("Prepare failed: ") + sqlite3_errmsg(db_);
      return;
    }

    sqlite3_bind_text(stmt, 1, sid.c_str(), -1, SQLITE_TRANSIENT);

    if (sqlite3_step(stmt) != SQLITE_DONE) {
      res->success = false;
      res->message = std::string("Update session failed: ") + sqlite3_errmsg(db_);
      sqlite3_finalize(stmt);
      return;
    }

    sqlite3_finalize(stmt);

    current_session_id_.clear();

    res->success = true;
    res->message = sid; // 종료한 session_id를 message로 반환(필요시)
    RCLCPP_INFO(get_logger(), "Session STOP: %s", sid.c_str());
  }

  // ===== service: get_heatmap (GetHeatmap.srv) =====
  void onGetHeatmap(
  const std::shared_ptr<wifi_interface::srv::GetHeatmap::Request> req,
  std::shared_ptr<wifi_interface::srv::GetHeatmap::Response> res)
{
  std::lock_guard<std::mutex> lk(db_mtx_);  // 먼저 락

  if (!req) {
    res->ok = false;
    res->message = "Null request";
    return;
  }
  if (req->session_id.empty()) {
    res->ok = false;
    res->message = "session_id is empty";
    return;
  }

  const int lim = (req->limit == 0) ? 50000 : (int)req->limit;  // ✅ req 체크 이후
  const int off = (int)req->offset;

    // SQL 구성
    std::string sql =
      "SELECT grid_id_x, grid_id_y, rssi_value, ssid, timestamp "
      "FROM wifi_data WHERE session_id = ? ";

    const bool ssid_filter = (!req->ssid.empty() && req->ssid != "ALL");
    if (ssid_filter) sql += "AND ssid = ? ";

    if (req->thr_enable) sql += "AND rssi_value >= ? ";

    sql += "ORDER BY id ASC LIMIT ? OFFSET ?;";

    sqlite3_stmt *stmt = nullptr;
    if (sqlite3_prepare_v2(db_, sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
      res->ok = false;
      res->message = std::string("Prepare failed: ") + sqlite3_errmsg(db_);
      return;
    }

    int idx = 1;
    sqlite3_bind_text(stmt, idx++, req->session_id.c_str(), -1, SQLITE_TRANSIENT);

    if (ssid_filter) {
      sqlite3_bind_text(stmt, idx++, req->ssid.c_str(), -1, SQLITE_TRANSIENT);
    }
    if (req->thr_enable) {
      sqlite3_bind_double(stmt, idx++, (double)req->thr_rssi);
    }

    sqlite3_bind_int(stmt, idx++, lim);
    sqlite3_bind_int(stmt, idx++, off);

    // 결과 채우기
    res->xs.clear(); res->ys.clear(); res->rssis.clear();
    res->ssids.clear(); res->stamps.clear();

    int count = 0;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      const double x = sqlite3_column_double(stmt, 0);
      const double y = sqlite3_column_double(stmt, 1);
      const double rssi = sqlite3_column_double(stmt, 2);

      const unsigned char* ssid_u = sqlite3_column_text(stmt, 3);
      const unsigned char* ts_u   = sqlite3_column_text(stmt, 4);

      res->xs.push_back(x);
      res->ys.push_back(y);
      res->rssis.push_back((int32_t)std::lround(rssi));
      res->ssids.push_back(ssid_u ? (const char*)ssid_u : "");
      res->stamps.push_back(ts_u ? (const char*)ts_u : "");

      ++count;
    }

    sqlite3_finalize(stmt);

    res->ok = true;
    res->message = "OK rows=" + std::to_string(count);
  }

  void onListSessions(
  const std::shared_ptr<wifi_interface::srv::ListSessions::Request> req,
  std::shared_ptr<wifi_interface::srv::ListSessions::Response> res)
 {
    std::lock_guard<std::mutex> lk(db_mtx_);
    if (!db_) {
      res->ok = false;
      res->message = "DB not opened";
      return;
  }

  // 최신 세션부터 내림차순
  const char* sql =
    "SELECT session_id, started_at, COALESCE(ended_at, '') "
    "FROM sessions "
    "ORDER BY started_at DESC "
    "LIMIT ? OFFSET ?;";

  sqlite3_stmt* stmt=nullptr;
  if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    res->ok = false;
    res->message = std::string("prepare failed: ") + sqlite3_errmsg(db_);
    return;
  }

  sqlite3_bind_int(stmt, 1, (int)req->limit);
  sqlite3_bind_int(stmt, 2, (int)req->offset);

  res->session_ids.clear();
  res->started_at.clear();
  res->ended_at.clear();

  while (sqlite3_step(stmt) == SQLITE_ROW) {
    const unsigned char* sid = sqlite3_column_text(stmt, 0);
    const unsigned char* st  = sqlite3_column_text(stmt, 1);
    const unsigned char* et  = sqlite3_column_text(stmt, 2);

    res->session_ids.push_back(sid ? (const char*)sid : "");
    res->started_at.push_back(st ? (const char*)st : "");
    res->ended_at.push_back(et ? (const char*)et : "");
  }

  sqlite3_finalize(stmt);

  res->ok = true;
  res->message = "OK";
}

void onDeleteSession(
  const std::shared_ptr<wifi_interface::srv::DeleteSession::Request> req,
  std::shared_ptr<wifi_interface::srv::DeleteSession::Response> res)
{
  std::lock_guard<std::mutex> lk(db_mtx_);

  if (!req || req->session_id.empty()) {
    res->ok = false;
    res->message = "session_id is empty";
    return;
  }

  // 현재 진행중 세션을 삭제하려면 먼저 stop 처리(정책)
  if (req->session_id == current_session_id_) {
    current_session_id_.clear();
  }

  // 트랜잭션(권장)
  sqlite3_exec(db_, "BEGIN;", nullptr, nullptr, nullptr);

  // wifi_data 삭제
  {
    const char* sql = "DELETE FROM wifi_data WHERE session_id = ?;";
    sqlite3_stmt* stmt=nullptr;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
      sqlite3_exec(db_, "ROLLBACK;", nullptr, nullptr, nullptr);
      res->ok=false; res->message = std::string("prepare failed: ") + sqlite3_errmsg(db_);
      return;
    }
    sqlite3_bind_text(stmt, 1, req->session_id.c_str(), -1, SQLITE_TRANSIENT);
    if (sqlite3_step(stmt) != SQLITE_DONE) {
      sqlite3_finalize(stmt);
      sqlite3_exec(db_, "ROLLBACK;", nullptr, nullptr, nullptr);
      res->ok=false; res->message = std::string("delete wifi_data failed: ") + sqlite3_errmsg(db_);
      return;
    }
    sqlite3_finalize(stmt);
  }

  // sessions 삭제
  {
    const char* sql = "DELETE FROM sessions WHERE session_id = ?;";
    sqlite3_stmt* stmt=nullptr;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
      sqlite3_exec(db_, "ROLLBACK;", nullptr, nullptr, nullptr);
      res->ok=false; res->message = std::string("prepare failed: ") + sqlite3_errmsg(db_);
      return;
    }
    sqlite3_bind_text(stmt, 1, req->session_id.c_str(), -1, SQLITE_TRANSIENT);
    if (sqlite3_step(stmt) != SQLITE_DONE) {
      sqlite3_finalize(stmt);
      sqlite3_exec(db_, "ROLLBACK;", nullptr, nullptr, nullptr);
      res->ok=false; res->message = std::string("delete sessions failed: ") + sqlite3_errmsg(db_);
      return;
    }
    sqlite3_finalize(stmt);
  }

  sqlite3_exec(db_, "COMMIT;", nullptr, nullptr, nullptr);

  res->ok = true;
  res->message = "Deleted: " + req->session_id;
}

void onListSsids(
  const std::shared_ptr<wifi_interface::srv::ListSsids::Request> req,
  std::shared_ptr<wifi_interface::srv::ListSsids::Response> res)
{
  std::lock_guard<std::mutex> lk(db_mtx_);

  if (!req || req->session_id.empty()) {
    res->ok = false;
    res->message = "session_id is empty";
    return;
  }

  const char* sql =
    "SELECT DISTINCT ssid FROM wifi_data "
    "WHERE session_id = ? "
    "ORDER BY ssid ASC "
    "LIMIT ? OFFSET ?;";

  sqlite3_stmt* stmt=nullptr;
  if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    res->ok=false;
    res->message = std::string("prepare failed: ") + sqlite3_errmsg(db_);
    return;
  }

  sqlite3_bind_text(stmt, 1, req->session_id.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_int(stmt, 2, (int)req->limit);
  sqlite3_bind_int(stmt, 3, (int)req->offset);

  res->ssids.clear();
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    const unsigned char* s = sqlite3_column_text(stmt, 0);
    res->ssids.push_back(s ? (const char*)s : "");
  }
  sqlite3_finalize(stmt);

  res->ok = true;
  res->message = "OK";
}


private:
  sqlite3 *db_{nullptr};
  std::mutex db_mtx_;

  std::string db_path_;
  std::string device_info_;

  // active session id in-memory
  std::string current_session_id_;

  rclcpp::Subscription<wifi_interface::msg::WifiFused>::SharedPtr sub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_stop_;
  rclcpp::Service<wifi_interface::srv::GetHeatmap>::SharedPtr srv_heatmap_;
  rclcpp::Service<wifi_interface::srv::ListSessions>::SharedPtr srv_list_sessions_;
  rclcpp::Service<wifi_interface::srv::DeleteSession>::SharedPtr srv_delete_;
  rclcpp::Service<wifi_interface::srv::ListSsids>::SharedPtr srv_list_ssids_;

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WifiDbNode>());
  rclcpp::shutdown();
  return 0;
}

