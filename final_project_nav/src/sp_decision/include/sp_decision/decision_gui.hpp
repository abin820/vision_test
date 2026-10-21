#ifndef SP_DECISION_DECISION_GUI_HPP_
#define SP_DECISION_DECISION_GUI_HPP_

#include <memory>
#include <string>

#include <QLabel>
#include <QListWidget>
#include <QMainWindow>
#include <QPushButton>

#include "sp_decision/decision_engine.hpp"

namespace sp_decision
{

class DecisionGui : public QMainWindow
{
  Q_OBJECT

public:
  explicit DecisionGui(
    std::shared_ptr<DecisionEngine> engine,
    QWidget * parent = nullptr);

  ~DecisionGui() override = default;

signals:

  void engine_status_signal(const QString & msg);

private slots:
  void on_start_clicked();
  void on_stop_clicked();
  void on_switch_clicked();
  void on_browse_clicked();
  void on_reload_dir_clicked();

  void update_status_display(const QString & msg);

private:
  void build_ui();
  void connect_signals();
  void refresh_tree_list();
  void refresh_status_panel();
  void populate_from_engine();

  std::shared_ptr<DecisionEngine> engine_;

  QListWidget  * tree_list_         {nullptr};
  QPushButton  * btn_start_         {nullptr};
  QPushButton  * btn_stop_          {nullptr};
  QPushButton  * btn_switch_        {nullptr};
  QPushButton  * btn_browse_        {nullptr};
  QPushButton  * btn_reload_        {nullptr};
  QLabel       * lbl_current_tree_  {nullptr};
  QLabel       * lbl_bt_status_     {nullptr};

  QString tree_dir_;
};

}

#endif
