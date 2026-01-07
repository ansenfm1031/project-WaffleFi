#include "wifi_interface/msg/wifi_fused.hpp"

void QtVisualizer::fusedCallback(
    const wifi_interface::msg::WifiFused::SharedPtr msg)
{
    QSqlQuery query;
    query.prepare(
        "INSERT INTO wifi_log (timestamp, x, y, rssi) "
        "VALUES (?, ?, ?, ?)"
    );
    query.addBindValue(msg->timestamp);
    query.addBindValue(msg->x);
    query.addBindValue(msg->y);
    query.addBindValue(msg->rssi);

    if (!query.exec()) {
        qDebug() << "SQLite insert error:" << query.lastError();
    }
}
