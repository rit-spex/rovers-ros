#include <algorithm>
#include <array>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

#include <gazebo/gui/GuiIface.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/gui/MainWindow.hh>
#include <gazebo/gui/qt.h>

#include <QAction>
#include <QApplication>
#include <QButtonGroup>
#include <QFileInfo>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QMenu>
#include <QProcess>
#include <QPushButton>
#include <QRadioButton>
#include <QToolButton>
#include <QVBoxLayout>
#include <QWidget>

namespace
{
struct ConfigOption
{
  std::string key;
  std::string label;
};

const std::array<ConfigOption, 3> kConfigurations = {{
    {"drivebase", "Drivebase only"},
    {"arm_drivebase", "Drivebase + Arm"},
    {"life_drivebase", "Drivebase + Life"}}};

std::string ConfigFile()
{
  const char *home = std::getenv("HOME");
  return std::string(home ? home : ".") + "/.config/rovers-ros/sim_config";
}

std::string LoadedConfig()
{
  std::ifstream file(ConfigFile());
  std::string value;
  if (file >> value)
  {
    for (const auto &config : kConfigurations)
    {
      if (config.key == value)
        return value;
    }
  }
  return "arm_drivebase";
}

std::string InitialConfig()
{
  const char *environment = std::getenv("ROBOT_CONFIG");
  if (environment && *environment)
  {
    for (const auto &config : kConfigurations)
    {
      if (config.key == environment)
        return environment;
    }
  }
  return LoadedConfig();
}

void SaveConfig(const std::string &key)
{
  const auto path = ConfigFile();
  std::filesystem::create_directories(
      std::filesystem::path(path).parent_path());
  std::ofstream file(path);
  if (file)
    file << key << "\n";
}

std::string Workspace()
{
  const char *workspace = std::getenv("ROVER_WS");
  if (workspace && *workspace)
    return workspace;

  // setup_gazebo() prepends <ws>/install/gazebo_sim_control/lib to
  // GAZEBO_PLUGIN_PATH, so that prefix identifies the workspace even when
  // ROVER_WS is not exported.
  const char *plugins = std::getenv("GAZEBO_PLUGIN_PATH");
  if (plugins && *plugins)
  {
    const std::string path(plugins);
    const auto install = path.find("/install/");
    if (install != std::string::npos)
      return path.substr(0, install);
  }

  // Fall back to the launch directory; RestartSimulation() reports a clear
  // error if scripts/restart_gazebo.sh is not found there.
  std::error_code error;
  const auto current = std::filesystem::current_path(error);
  return error ? std::string(".") : current.string();
}
}  // namespace

namespace gazebo
{
class SimControlGui : public GUIPlugin
{
  public: SimControlGui();
  public: virtual ~SimControlGui();
  public: virtual void Load(sdf::ElementPtr _sdf) override;

  private: void RestartSimulation(const std::string &key);
  private: void SetStatus(const QString &text);
  private: void SetCollapsed(bool collapsed);
  private: bool eventFilter(QObject *watched, QEvent *event) override;

  private: QWidget *contentWidget{nullptr};
  private: QLabel *statusLabel{nullptr};
  private: QWidget *headerWidget{nullptr};
  private: QToolButton *collapseButton{nullptr};
  private: QAction *toggleAction{nullptr};
  private: int expandedWidth{360};
  private: int expandedHeight{300};
  private: bool collapsed{false};
  private: std::string selectedConfig{"arm_drivebase"};
};

SimControlGui::SimControlGui()
{
  this->setObjectName("roverSimControlGui");
  this->setMinimumSize(360, 300);
  this->resize(360, 300);
  this->move(10, 60);
  this->setAutoFillBackground(true);
  QPalette palette = this->palette();
  palette.setColor(QPalette::Window, QColor(238, 240, 244));
  this->setPalette(palette);

  qApp->installEventFilter(this);

  auto *mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(10, 10, 10, 10);
  mainLayout->setSpacing(8);

  auto *header = new QWidget(this);
  this->headerWidget = header;
  auto *headerLayout = new QHBoxLayout(header);
  headerLayout->setContentsMargins(0, 0, 0, 0);
  headerLayout->setSpacing(6);

  auto *title = new QLabel("SIMULATION CONTROL", header);
  QFont titleFont = title->font();
  titleFont.setBold(true);
  titleFont.setPointSize(titleFont.pointSize() + 2);
  title->setFont(titleFont);
  headerLayout->addWidget(title, 1);

  auto *shortcutHint = new QLabel("Ctrl+Shift+C", header);
  QFont hintFont = shortcutHint->font();
  hintFont.setPointSize(std::max(7, hintFont.pointSize() - 2));
  shortcutHint->setFont(hintFont);
  shortcutHint->setStyleSheet("color: #777777;");
  headerLayout->addWidget(shortcutHint, 0, Qt::AlignVCenter);

  this->collapseButton = new QToolButton(header);
  this->collapseButton->setArrowType(Qt::DownArrow);
  this->collapseButton->setAutoRaise(true);
  this->collapseButton->setToolTip(
      "Collapse simulation control (Ctrl+Shift+C)");
  this->collapseButton->setFixedSize(22, 22);
  headerLayout->addWidget(this->collapseButton, 0, Qt::AlignTop);
  connect(this->collapseButton, &QToolButton::clicked, this,
      [this]() { this->SetCollapsed(!this->collapsed); });
  mainLayout->addWidget(header);

  auto *content = new QWidget(this);
  this->contentWidget = content;
  auto *contentLayout = new QVBoxLayout(content);
  contentLayout->setContentsMargins(0, 0, 0, 0);
  contentLayout->setSpacing(8);

  auto *configGroup = new QGroupBox("Robot configuration", content);
  auto *configLayout = new QVBoxLayout(configGroup);
  auto *configButtons = new QButtonGroup(configGroup);
  const std::string savedConfig = InitialConfig();
  this->selectedConfig = savedConfig;
  std::size_t configIndex = 0;
  for (const auto &config : kConfigurations)
  {
    auto *button = new QRadioButton(
        QString::fromStdString(config.label), configGroup);
    button->setProperty("configKey", QString::fromStdString(config.key));
    configButtons->addButton(button, static_cast<int>(configIndex));
    configLayout->addWidget(button);
    if (config.key == savedConfig)
      button->setChecked(true);
    ++configIndex;
  }
  connect(configButtons,
      QOverload<QAbstractButton *, bool>::of(&QButtonGroup::buttonToggled),
      this, [this](QAbstractButton *button, bool checked)
      {
        if (!checked)
          return;
        const std::string key =
            button->property("configKey").toString().toStdString();
        this->selectedConfig = key;
        SaveConfig(key);
        this->SetStatus(
            QString("Saved '%1'. Click Restart simulation to apply.")
            .arg(QString::fromStdString(key)));
      });
  contentLayout->addWidget(configGroup);

  auto *restartButton = new QPushButton("Restart simulation", content);
  contentLayout->addWidget(restartButton);
  connect(restartButton, &QPushButton::clicked, this, [this]()
      {
        this->RestartSimulation(this->selectedConfig);
      });
  contentLayout->addStretch(1);

  mainLayout->addWidget(content, 1);

  this->statusLabel = new QLabel(
      "Configuration is saved for the next launch.", this);
  this->statusLabel->setWordWrap(true);
  mainLayout->addWidget(this->statusLabel);

  this->setLayout(mainLayout);
  this->show();
}

SimControlGui::~SimControlGui() = default;

void SimControlGui::Load(sdf::ElementPtr /*_sdf*/)
{
  if (this->toggleAction)
    return;
  auto *menu = new QMenu("Simulation", this);
  this->toggleAction = menu->addAction("Collapse control panel");
  connect(this->toggleAction, &QAction::triggered, this,
      [this]() { this->SetCollapsed(!this->collapsed); });
  if (auto *mainWindow = gazebo::gui::get_main_window())
    mainWindow->AddMenu(menu);
}

void SimControlGui::RestartSimulation(const std::string &key)
{
  const QString script =
      QString::fromStdString(Workspace()) + "/scripts/restart_gazebo.sh";
  if (!QFileInfo::exists(script))
  {
    this->SetStatus(QString("Restart script not found: %1").arg(script));
    return;
  }
  SaveConfig(key);
  QProcess::startDetached(
      "bash", {script, QString::fromStdString(key)});
  this->SetStatus(QString("Restarting Gazebo with '%1'...")
      .arg(QString::fromStdString(key)));
}

bool SimControlGui::eventFilter(QObject *watched, QEvent *event)
{
  // Collapse/expand from anywhere in the Gazebo window. The panel can be
  // covered by native Gazebo child windows, so a keyboard shortcut is the
  // reliable way to toggle it.
  if (event->type() == QEvent::KeyPress)
  {
    auto *shortcut = static_cast<QKeyEvent *>(event);
    if (shortcut->key() == Qt::Key_C &&
        shortcut->modifiers().testFlag(Qt::ControlModifier) &&
        shortcut->modifiers().testFlag(Qt::ShiftModifier))
    {
      this->SetCollapsed(!this->collapsed);
      return true;
    }
  }
  return QWidget::eventFilter(watched, event);
}

void SimControlGui::SetCollapsed(bool collapse)
{
  if (this->collapsed == collapse)
    return;

  const QMargins margins = this->layout()->contentsMargins();
  this->collapsed = collapse;
  if (collapse)
  {
    this->expandedWidth = this->width();
    this->expandedHeight = this->height();
    this->contentWidget->hide();
    this->statusLabel->hide();
    this->collapseButton->setArrowType(Qt::RightArrow);
    this->collapseButton->setToolTip(
        "Expand simulation control (Ctrl+Shift+C)");
    if (this->toggleAction)
      this->toggleAction->setText("Expand control panel");

    const int headerWidth = this->headerWidget->sizeHint().width()
        + margins.left() + margins.right();
    const int headerHeight = this->headerWidget->sizeHint().height()
        + margins.top() + margins.bottom();
    this->setMinimumSize(headerWidth, headerHeight);
    this->setMaximumSize(headerWidth, headerHeight);
    this->resize(headerWidth, headerHeight);
  }
  else
  {
    this->setMinimumSize(360, 300);
    this->setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
    this->contentWidget->show();
    this->statusLabel->show();
    this->collapseButton->setArrowType(Qt::DownArrow);
    this->collapseButton->setToolTip(
        "Collapse simulation control (Ctrl+Shift+C)");
    if (this->toggleAction)
      this->toggleAction->setText("Collapse control panel");
    this->resize(this->expandedWidth, this->expandedHeight);
  }
}

void SimControlGui::SetStatus(const QString &text)
{
  if (this->statusLabel)
    this->statusLabel->setText(text);
}
}  // namespace gazebo

using gazebo::SimControlGui;

GZ_REGISTER_GUI_PLUGIN(SimControlGui)
