#include "selfdrive/ui/qt/offroad/settings.h"

#include <cassert>
#include <cmath>
#include <string>
#include <tuple>
#include <vector>

#include <QDebug>

#include "selfdrive/ui/qt/network/networking.h"

#include "common/params.h"
#include "common/watchdog.h"
#include "common/util.h"
#include "system/hardware/hw.h"
#include "selfdrive/ui/qt/widgets/controls.h"
#include "selfdrive/ui/qt/widgets/input.h"
#include "selfdrive/ui/qt/widgets/scrollview.h"
#include "selfdrive/ui/qt/widgets/ssh_keys.h"
#include "selfdrive/ui/qt/widgets/toggle.h"
#include "selfdrive/ui/ui.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/qt_window.h"

#include "selfdrive/frogpilot/ui/frogpilot_settings.h"
#include "selfdrive/frogpilot/ui/vehicle_settings.h"
#include "selfdrive/frogpilot/navigation/ui/navigation_settings.h"

TogglesPanel::TogglesPanel(SettingsWindow *parent) : ListWidget(parent) {
  // param, title, desc, icon
  std::vector<std::tuple<QString, QString, QString, QString>> toggle_defs{
    {
      "OpenpilotEnabledToggle",
      tr("Enable openpilot"),
      tr("Use the openpilot system for adaptive cruise control and lane keep driver assistance. Your attention is required at all times to use this feature. Changing this setting takes effect when the car is powered off."),
      "../assets/offroad/icon_openpilot.png",
    },
    {
      "ExperimentalLongitudinalEnabled",
      tr("openpilot Longitudinal Control (Alpha)"),
      QString("<b>%1</b><br><br>%2")
      .arg(tr("WARNING: openpilot longitudinal control is in alpha for this car and will disable Automatic Emergency Braking (AEB)."))
      .arg(tr("On this car, openpilot defaults to the car's built-in ACC instead of openpilot's longitudinal control. "
              "Enable this to switch to openpilot longitudinal control. Enabling Experimental mode is recommended when enabling openpilot longitudinal control alpha.")),
      "../assets/offroad/icon_speed_limit.png",
    },
    {
      "ExperimentalMode",
      tr("Experimental Mode"),
      "",
      "../assets/img_experimental_white.svg",
    },
    {
      "DisengageOnAccelerator",
      tr("Disengage on Accelerator Pedal"),
      tr("When enabled, pressing the accelerator pedal will disengage openpilot."),
      "../assets/offroad/icon_disengage_on_accelerator.svg",
    },
    {
      "IsLdwEnabled",
      tr("Enable Lane Departure Warnings"),
      tr("Receive alerts to steer back into the lane when your vehicle drifts over a detected lane line without a turn signal activated while driving over 31 mph (50 km/h)."),
      "../assets/offroad/icon_warning.png",
    },
    {
      "RecordFront",
      tr("Record and Upload Driver Camera"),
      tr("Upload data from the driver facing camera and help improve the driver monitoring algorithm."),
      "../assets/offroad/icon_monitoring.png",
    },
    {
      "IsMetric",
      tr("Use Metric System"),
      tr("Display speed in km/h instead of mph."),
      "../assets/offroad/icon_metric.png",
    },
#ifdef ENABLE_MAPS
    {
      "NavSettingTime24h",
      tr("Show ETA in 24h Format"),
      tr("Use 24h format instead of am/pm"),
      "../assets/offroad/icon_metric.png",
    },
    {
      "NavSettingLeftSide",
      tr("Show Map on Left Side of UI"),
      tr("Show map on left side when in split screen view."),
      "../assets/offroad/icon_road.png",
    },
#endif
  };


  std::vector<QString> longi_button_texts{tr("Aggressive"), tr("Standard"), tr("Relaxed")};
  long_personality_setting = new ButtonParamControl("LongitudinalPersonality", tr("Driving Personality"),
                                          tr("Standard is recommended. In aggressive mode, openpilot will follow lead cars closer and be more aggressive with the gas and brake. "
                                             "In relaxed mode openpilot will stay further away from lead cars."),
                                          "../assets/offroad/icon_speed_limit.png",
                                          longi_button_texts);
  for (auto &[param, title, desc, icon] : toggle_defs) {
    auto toggle = new ParamControl(param, title, desc, icon, this);

    bool locked = params.getBool((param + "Lock").toStdString());
    toggle->setEnabled(!locked);

    addItem(toggle);
    toggles[param.toStdString()] = toggle;

    // insert longitudinal personality after NDOG toggle
    if (param == "DisengageOnAccelerator" && !params.getInt("AdjustablePersonalities")) {
      addItem(long_personality_setting);
    }
  }

  // Toggles with confirmation dialogs
  toggles["ExperimentalMode"]->setActiveIcon("../assets/img_experimental.svg");
  toggles["ExperimentalMode"]->setConfirmation(true, true);
  toggles["ExperimentalLongitudinalEnabled"]->setConfirmation(true, false);

  connect(toggles["ExperimentalLongitudinalEnabled"], &ToggleControl::toggleFlipped, [=]() {
    updateToggles();
  });
}

void TogglesPanel::expandToggleDescription(const QString &param) {
  toggles[param.toStdString()]->showDescription();
}

void TogglesPanel::showEvent(QShowEvent *event) {
  updateToggles();
}

void TogglesPanel::updateToggles() {
  auto experimental_mode_toggle = toggles["ExperimentalMode"];
  auto op_long_toggle = toggles["ExperimentalLongitudinalEnabled"];
  const QString e2e_description = QString("%1<br>"
                                          "<h4>%2</h4><br>"
                                          "%3<br>"
                                          "<h4>%4</h4><br>"
                                          "%5<br>"
                                          "<h4>%6</h4><br>"
                                          "%7")
                                  .arg(tr("openpilot defaults to driving in <b>chill mode</b>. Experimental mode enables <b>alpha-level features</b> that aren't ready for chill mode. Experimental features are listed below:"))
                                  .arg(tr("End-to-End Longitudinal Control"))
                                  .arg(tr("Let the driving model control the gas and brakes. openpilot will drive as it thinks a human would, including stopping for red lights and stop signs. "
                                          "Since the driving model decides the speed to drive, the set speed will only act as an upper bound. This is an alpha quality feature; "
                                          "mistakes should be expected."))
                                  .arg(tr("Navigate on openpilot"))
                                  .arg(tr("When navigation has a destination, openpilot will input the map information into the model. This provides useful context for the model and allows openpilot to keep left or right "
                                          "appropriately at forks/exits. Lane change behavior is unchanged and still activated by the driver. This is an alpha quality feature; mistakes should be expected, particularly around "
                                          "exits and forks. These mistakes can include unintended laneline crossings, late exit taking, driving towards dividing barriers in the gore areas, etc."))
                                  .arg(tr("New Driving Visualization"))
                                  .arg(tr("The driving visualization will transition to the road-facing wide-angle camera at low speeds to better show some turns. The Experimental mode logo will also be shown in the top right corner. "
                                          "When a navigation destination is set and the driving model is using it as input, the driving path on the map will turn green."));

  const bool is_release = params.getBool("IsReleaseBranch");
  auto cp_bytes = params.get("CarParamsPersistent");
  if (!cp_bytes.empty()) {
    AlignedBuffer aligned_buf;
    capnp::FlatArrayMessageReader cmsg(aligned_buf.align(cp_bytes.data(), cp_bytes.size()));
    cereal::CarParams::Reader CP = cmsg.getRoot<cereal::CarParams>();

    if (!CP.getExperimentalLongitudinalAvailable() || is_release) {
      params.remove("ExperimentalLongitudinalEnabled");
    }
    op_long_toggle->setVisible(CP.getExperimentalLongitudinalAvailable() && !is_release);
    if (hasLongitudinalControl(CP)) {
      // normal description and toggle
      experimental_mode_toggle->setEnabled(!params.getBool("ConditionalExperimental"));
      experimental_mode_toggle->setDescription(e2e_description);
      long_personality_setting->setEnabled(true);
    } else {
      // no long for now
      experimental_mode_toggle->setEnabled(false);
      long_personality_setting->setEnabled(false);
      params.remove("ExperimentalMode");

      const QString unavailable = tr("Experimental mode is currently unavailable on this car since the car's stock ACC is used for longitudinal control.");

      QString long_desc = unavailable + " " + \
                          tr("openpilot longitudinal control may come in a future update.");
      if (CP.getExperimentalLongitudinalAvailable()) {
        if (is_release) {
          long_desc = unavailable + " " + tr("An alpha version of openpilot longitudinal control can be tested, along with Experimental mode, on non-release branches.");
        } else {
          long_desc = tr("Enable the openpilot longitudinal control (alpha) toggle to allow Experimental mode.");
        }
      }
      experimental_mode_toggle->setDescription("<b>" + long_desc + "</b><br><br>" + e2e_description);
    }

    experimental_mode_toggle->refresh();
  } else {
    experimental_mode_toggle->setDescription(e2e_description);
    op_long_toggle->setVisible(false);
  }
}

DevicePanel::DevicePanel(SettingsWindow *parent) : ListWidget(parent) {
  setSpacing(50);
  addItem(new LabelControl(tr("Dongle ID"), getDongleId().value_or(tr("N/A"))));
  addItem(new LabelControl(tr("Serial"), params.get("HardwareSerial").c_str()));

  // offroad-only buttons

  auto dcamBtn = new ButtonControl(tr("Driver Camera"), tr("PREVIEW"),
                                   tr("Preview the driver facing camera to ensure that driver monitoring has good visibility. (vehicle must be off)"));
  connect(dcamBtn, &ButtonControl::clicked, [=]() { emit showDriverView(); });
  addItem(dcamBtn);

  auto resetCalibBtn = new ButtonControl(tr("Reset Calibration"), tr("RESET"), "");
  connect(resetCalibBtn, &ButtonControl::showDescriptionEvent, this, &DevicePanel::updateCalibDescription);
  connect(resetCalibBtn, &ButtonControl::clicked, [&]() {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to reset calibration?"), tr("Reset"), this)) {
      params.remove("CalibrationParams");
      params.remove("LiveTorqueParameters");
    }
  });
  addItem(resetCalibBtn);

  if (!params.getBool("Passive")) {
    auto retrainingBtn = new ButtonControl(tr("Review Training Guide"), tr("REVIEW"), tr("Review the rules, features, and limitations of openpilot"));
    connect(retrainingBtn, &ButtonControl::clicked, [=]() {
      if (ConfirmationDialog::confirm(tr("Are you sure you want to review the training guide?"), tr("Review"), this)) {
        emit reviewTrainingGuide();
      }
    });
    addItem(retrainingBtn);
  }

  if (Hardware::TICI()) {
    auto regulatoryBtn = new ButtonControl(tr("Regulatory"), tr("VIEW"), "");
    connect(regulatoryBtn, &ButtonControl::clicked, [=]() {
      const std::string txt = util::read_file("../assets/offroad/fcc.html");
      ConfirmationDialog::rich(QString::fromStdString(txt), this);
    });
    addItem(regulatoryBtn);
  }

  auto translateBtn = new ButtonControl(tr("Change Language"), tr("CHANGE"), "");
  connect(translateBtn, &ButtonControl::clicked, [=]() {
    QMap<QString, QString> langs = getSupportedLanguages();
    QString selection = MultiOptionDialog::getSelection(tr("Select a language"), langs.keys(), langs.key(uiState()->language), this);
    if (!selection.isEmpty()) {
      // put language setting, exit Qt UI, and trigger fast restart
      params.put("LanguageSetting", langs[selection].toStdString());
      qApp->exit(18);
      watchdog_kick(0);
    }
  });
  addItem(translateBtn);

  // Panda flashing button
  const auto flashPandaBtn = new ButtonControl(tr("Flash Panda"), tr("FLASH"), "");
  connect(flashPandaBtn, &ButtonControl::clicked, [this]() {
    if (!ConfirmationDialog::confirm(tr("Are you sure you want to flash the Panda?"), tr("Flash"), this)) return;
    QProcess process;
    // Get Panda type
    const SubMaster &sm = *(uiState()->sm);
    const auto pandaStates = sm["pandaStates"].getPandaStates();
    // Choose recovery script based on Panda type
    if (pandaStates.size() != 0) {
      const auto pandaType = pandaStates[0].getPandaType();
      const bool isRedPanda = (pandaType == cereal::PandaState::PandaType::RED_PANDA ||
                               pandaType == cereal::PandaState::PandaType::RED_PANDA_V2);
      const QString recoveryScript = isRedPanda ? "./recover.sh" : "./recover.py";
      // Run recovery script and flash Panda
      process.setWorkingDirectory("/data/openpilot/panda/board");
      process.start("/bin/sh", QStringList{"-c", recoveryScript});
      process.waitForFinished();
    }
    // Run the killall script as a redundancy
    process.setWorkingDirectory("/data/openpilot/panda");
    process.start("/bin/sh", QStringList{"-c", "pkill -f boardd; PYTHONPATH=.. python -c \"from panda import Panda; Panda().flash()\""});
    process.waitForFinished();
    Hardware::reboot();
  });
  addItem(flashPandaBtn);

  QObject::connect(uiState(), &UIState::offroadTransition, [=](bool offroad) {
    for (auto btn : findChildren<ButtonControl *>()) {
      btn->setEnabled(offroad);
    }
  });

  // power buttons
  QHBoxLayout *power_layout = new QHBoxLayout();
  power_layout->setSpacing(30);

  QPushButton *reboot_btn = new QPushButton(tr("Reboot"));
  reboot_btn->setObjectName("reboot_btn");
  power_layout->addWidget(reboot_btn);
  QObject::connect(reboot_btn, &QPushButton::clicked, this, &DevicePanel::reboot);

  QPushButton *poweroff_btn = new QPushButton(tr("Power Off"));
  poweroff_btn->setObjectName("poweroff_btn");
  power_layout->addWidget(poweroff_btn);
  QObject::connect(poweroff_btn, &QPushButton::clicked, this, &DevicePanel::poweroff);

  if (!Hardware::PC()) {
    connect(uiState(), &UIState::offroadTransition, poweroff_btn, &QPushButton::setVisible);
  }

  setStyleSheet(R"(
    #reboot_btn { height: 120px; border-radius: 15px; background-color: #393939; }
    #reboot_btn:pressed { background-color: #4a4a4a; }
    #poweroff_btn { height: 120px; border-radius: 15px; background-color: #E22C2C; }
    #poweroff_btn:pressed { background-color: #FF2424; }
  )");
  addItem(power_layout);
}

void DevicePanel::updateCalibDescription() {
  QString desc =
      tr("openpilot requires the device to be mounted within 4° left or right and "
         "within 5° up or 9° down. openpilot is continuously calibrating, resetting is rarely required.");
  std::string calib_bytes = params.get("CalibrationParams");
  if (!calib_bytes.empty()) {
    try {
      AlignedBuffer aligned_buf;
      capnp::FlatArrayMessageReader cmsg(aligned_buf.align(calib_bytes.data(), calib_bytes.size()));
      auto calib = cmsg.getRoot<cereal::Event>().getLiveCalibration();
      if (calib.getCalStatus() != cereal::LiveCalibrationData::Status::UNCALIBRATED) {
        double pitch = calib.getRpyCalib()[1] * (180 / M_PI);
        double yaw = calib.getRpyCalib()[2] * (180 / M_PI);
        desc += tr(" Your device is pointed %1° %2 and %3° %4.")
                    .arg(QString::number(std::abs(pitch), 'g', 1), pitch > 0 ? tr("down") : tr("up"),
                         QString::number(std::abs(yaw), 'g', 1), yaw > 0 ? tr("left") : tr("right"));
      }
    } catch (kj::Exception) {
      qInfo() << "invalid CalibrationParams";
    }
  }
  qobject_cast<ButtonControl *>(sender())->setDescription(desc);
}

void DevicePanel::reboot() {
  if (!uiState()->engaged()) {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to reboot?"), tr("Reboot"), this)) {
      // Check engaged again in case it changed while the dialog was open
      if (!uiState()->engaged()) {
        params.putBool("DoReboot", true);
      }
    }
  } else {
    ConfirmationDialog::alert(tr("Disengage to Reboot"), this);
  }
}

void DevicePanel::poweroff() {
  if (!uiState()->engaged()) {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to power off?"), tr("Power Off"), this)) {
      // Check engaged again in case it changed while the dialog was open
      if (!uiState()->engaged()) {
        params.putBool("DoShutdown", true);
      }
    }
  } else {
    ConfirmationDialog::alert(tr("Disengage to Power Off"), this);
  }
}

void SettingsWindow::showEvent(QShowEvent *event) {
  setCurrentPanel(0);
}

void SettingsWindow::setCurrentPanel(int index, const QString &param) {
  panel_widget->setCurrentIndex(index);
  nav_btns->buttons()[index]->setChecked(true);
  if (!param.isEmpty()) {
    emit expandToggleDescription(param);
  }
}

SettingsWindow::SettingsWindow(QWidget *parent) : QFrame(parent) {

  // setup two main layouts
  sidebar_widget = new QWidget;
  QVBoxLayout *sidebar_layout = new QVBoxLayout(sidebar_widget);
  sidebar_layout->setMargin(0);
  panel_widget = new QStackedWidget();

  // close button
  QPushButton *close_btn = new QPushButton(tr("← Back"));
  close_btn->setStyleSheet(R"(
    QPushButton {
      font-size: 50px;
      padding-bottom: 0px;
      border 1px grey solid;
      border-radius: 25px;
      background-color: #292929;
      font-weight: 500;
    }
    QPushButton:pressed {
      background-color: #3B3B3B;
    }
  )");
  close_btn->setFixedSize(300, 125);
  sidebar_layout->addSpacing(10);
  sidebar_layout->addWidget(close_btn, 0, Qt::AlignRight);
  QObject::connect(close_btn, &QPushButton::clicked, this, &SettingsWindow::closeSettings);

  // setup panels
  DevicePanel *device = new DevicePanel(this);
  QObject::connect(device, &DevicePanel::reviewTrainingGuide, this, &SettingsWindow::reviewTrainingGuide);
  QObject::connect(device, &DevicePanel::showDriverView, this, &SettingsWindow::showDriverView);

  TogglesPanel *toggles = new TogglesPanel(this);
  QObject::connect(this, &SettingsWindow::expandToggleDescription, toggles, &TogglesPanel::expandToggleDescription);

  QList<QPair<QString, QWidget *>> panels = {
    {tr("Device"), device},
    {tr("Network"), new Networking(this)},
    {tr("Toggles"), toggles},
    {tr("Software"), new SoftwarePanel(this)},
    {tr("Carrot"), new CarrotPanel(this)},
    {tr("Controls"), new FrogPilotControlsPanel(this)},
    {tr("Navigation"), new FrogPilotNavigationPanel(this)},
    {tr("Vehicles"), new FrogPilotVehiclesPanel(this)},
    {tr("Visuals"), new FrogPilotVisualsPanel(this)},
  };

  nav_btns = new QButtonGroup(this);
  for (auto &[name, panel] : panels) {
    QPushButton *btn = new QPushButton(name);
    btn->setCheckable(true);
    btn->setChecked(nav_btns->buttons().size() == 0);
    btn->setStyleSheet(R"(
      QPushButton {
        color: grey;
        border: none;
        background: none;
        font-size: 65px;
        font-weight: 500;
      }
      QPushButton:checked {
        color: white;
      }
      QPushButton:pressed {
        color: #ADADAD;
      }
    )");
    btn->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    nav_btns->addButton(btn);
    sidebar_layout->addWidget(btn, 0, Qt::AlignRight);

    const int lr_margin = name != tr("Network") ? 50 : 0;  // Network panel handles its own margins
    panel->setContentsMargins(lr_margin, 25, lr_margin, 25);

    ScrollView *panel_frame = new ScrollView(panel, this);
    panel_widget->addWidget(panel_frame);

    QObject::connect(btn, &QPushButton::clicked, [=, w = panel_frame]() {
      btn->setChecked(true);
      panel_widget->setCurrentWidget(w);
    });
  }
  sidebar_layout->setContentsMargins(50, 50, 100, 50);

  // main settings layout, sidebar + main panel
  QHBoxLayout *main_layout = new QHBoxLayout(this);

  sidebar_widget->setFixedWidth(500);
  main_layout->addWidget(sidebar_widget);
  main_layout->addWidget(panel_widget);

  setStyleSheet(R"(
    * {
      color: white;
      font-size: 50px;
    }
    SettingsWindow {
      background-color: black;
    }
    QStackedWidget, ScrollView {
      background-color: #292929;
      border-radius: 30px;
    }
  )");
}

CarrotPanel::CarrotPanel(QWidget* parent) : QWidget(parent) {

    main_layout = new QStackedLayout(this);
    homeScreen = new QWidget(this);
    carrotLayout = new QVBoxLayout(homeScreen);
    carrotLayout->setMargin(40);

    QHBoxLayout* select_layout = new QHBoxLayout();
    select_layout->setSpacing(30);

    QPushButton* cruise_btn = new QPushButton(tr("Cruise"));
    cruise_btn->setObjectName("cruise_btn");
    QObject::connect(cruise_btn, &QPushButton::clicked, this, [this]() {this->togglesCarrot(0); });

    QPushButton* latLong_btn = new QPushButton(tr("Tuning"));
    latLong_btn->setObjectName("latLong_btn");
    QObject::connect(latLong_btn, &QPushButton::clicked, this, [this]() {this->togglesCarrot(1); });

    QPushButton* disp_btn = new QPushButton(tr("Disp"));
    disp_btn->setObjectName("disp_btn");
    QObject::connect(disp_btn, &QPushButton::clicked, this, [this]() {this->togglesCarrot(2); });

    QPushButton* path_btn = new QPushButton(tr("Path"));
    path_btn->setObjectName("path_btn");
    QObject::connect(path_btn, &QPushButton::clicked, this, [this]() {this->togglesCarrot(3); });

    QPushButton* start_btn = new QPushButton(tr("Start"));
    start_btn->setObjectName("start_btn");
    QObject::connect(start_btn, &QPushButton::clicked, this, [this]() {this->togglesCarrot(4); });

    QPushButton* speed_btn = new QPushButton(tr("Speed"));
    speed_btn->setObjectName("speed_btn");
    QObject::connect(speed_btn, &QPushButton::clicked, this, [this]() {this->togglesCarrot(5); });

    select_layout->addWidget(start_btn);
    select_layout->addWidget(cruise_btn);
    select_layout->addWidget(speed_btn);
    select_layout->addWidget(latLong_btn);
    select_layout->addWidget(disp_btn);
    select_layout->addWidget(path_btn);
    carrotLayout->addLayout(select_layout, 0);

    QWidget* toggles = new QWidget();
    QVBoxLayout* toggles_layout = new QVBoxLayout(toggles);

    cruiseToggles = new ListWidget(this);
    cruiseToggles->addItem(new CValueControl("AutoCruiseControl", "(HKG) Auto Cruise control", "Softhold, Auto Cruise ON/OFF control", "../assets/offroad/icon_road.png", 0, 1, 1));
    cruiseToggles->addItem(new CValueControl("TrafficStopMode", "STOPPING: Traffice Stop Mode (1)", "0:Not Used, 1:Use", "../assets/offroad/icon_road.png", 0, 1, 1));
    cruiseToggles->addItem(new CValueControl("CruiseEcoControl", "CRUISE: Eco control(4km/h)", "Temporarily increasing the set speed to improve fuel efficiency.", "../assets/offroad/icon_road.png", 0, 10, 1));
    cruiseToggles->addItem(new CValueControl("CruiseOnDist", "CRUISE: Auto ON distance(0cm)", "When GAS/Brake is OFF, Cruise ON when the lead car gets closer or warning (- value).", "../assets/offroad/icon_road.png", -500, 500, 50));
    cruiseToggles->addItem(new CValueControl("CruiseSpeedUnit", "Button: Cruise Speed Unit", "", "../assets/offroad/icon_road.png", 1, 20, 1));
    cruiseToggles->addItem(new CValueControl("CruiseSpeedMin", "Cruise Speed: Lower limit(10)", "Cruise control MIN speed", "../assets/offroad/icon_road.png", 5, 50, 1));
    //cruiseToggles->addItem(new CValueControl("TFollowSpeedAddM", "GAP: Additinal TFs 40km/h(0)x0.01s", "Speed-dependent additinal max(100km/h) TFs", "../assets/offroad/icon_road.png", -100, 200, 5));
    cruiseToggles->addItem(new CValueControl("TFollowSpeedAdd", "GAP: Additinal TFs 100Km/h(0)x0.01s", "Speed-dependent additinal max(100km/h) TFs", "../assets/offroad/icon_road.png", -100, 200, 5));
    cruiseToggles->addItem(new CValueControl("MyDrivingMode", "DRIVEMODE: Select", "1:ECO,2:SAFE,3:NORMAL,4:HIGH", "../assets/offroad/icon_road.png", 1, 4, 1));
    cruiseToggles->addItem(new CValueControl("MyEcoModeFactor", "DRIVEMODE: ECO Accel ratio(80%)", "Acceleartion ratio in ECO mode", "../assets/offroad/icon_road.png", 10, 95, 5));
    cruiseToggles->addItem(new CValueControl("MySafeModeFactor", "DRIVEMODE: SAFE ratio(80%)", "Accel/StopDistance/DecelRatio/Gap control ratio", "../assets/offroad/icon_road.png", 10, 90, 10));

    latLongToggles = new ListWidget(this);
    latLongToggles->addItem(new CValueControl("UseLaneLineSpeed", "UseLaneLine Speed KPH(0)", "Above set speed: laneline. Below: laneless mode.", "../assets/offroad/icon_shell.png", 0, 200, 5));
    latLongToggles->addItem(new CValueControl("PathOffset", "PathOffset", "(-)left, (+)right, when UseLaneLineSpeed > 0", "../assets/offroad/icon_road.png", -50, 50, 1));
    latLongToggles->addItem(new CValueControl("LiveSteerRatioApply", "LAT: LiveSteerRatioApply(100)", "오버스티어가 발생하면 줄입니다.", "../assets/offroad/icon_road.png", 50, 110, 1));
    latLongToggles->addItem(new CValueControl("LiveTorqueCache", "LAT: LiveTorqueCache(0)", "주기적으로 LiveTorqueParameter를 저장.", "../assets/offroad/icon_road.png", 0, 1, 1));
    latLongToggles->addItem(new CValueControl("LateralTorqueCustom", "LAT: TorqueCustom(0)", "", "../assets/offroad/icon_road.png", 0, 2, 1));
    latLongToggles->addItem(new CValueControl("LateralTorqueAccelFactor", "LAT: TorqueAccelFactor(2500)", "", "../assets/offroad/icon_road.png", 1000, 4000, 10));
    latLongToggles->addItem(new CValueControl("LateralTorqueFriction", "LAT: TorqueFriction(100)", "", "../assets/offroad/icon_road.png", 0, 1000, 10));
    //latLongToggles->addItem(horizontal_line());
    //latLongToggles->addItem(new CValueControl("JerkStartLimit", "LONG: JERK START(10)x0.1", "Starting Jerk.", "../assets/offroad/icon_road.png", 1, 50, 1));
    latLongToggles->addItem(new CValueControl("LongitudinalTuningKpV", "LONG: P Gain(100)", "", "../assets/offroad/icon_road.png", 50, 150, 1));
    latLongToggles->addItem(new CValueControl("LongitudinalTuningKiV", "LONG: I Gain(200)", "", "../assets/offroad/icon_road.png", 0, 2000, 5));
    latLongToggles->addItem(new CValueControl("LongitudinalTuningKf", "LONG: FF Gain(200)", "", "../assets/offroad/icon_road.png", 0, 200, 1));
    latLongToggles->addItem(new CValueControl("StartAccelApply", "LONG: StartingAccel 2.0x(0%)", "정지->출발시 가속도의 가속율을 지정합니다 0: 사용안함.", "../assets/offroad/icon_road.png", 0, 100, 10));
    latLongToggles->addItem(new CValueControl("StopAccelApply", "LONG: StoppingAccel -2.0x(50%)", "정지유지시 브레이크압을 조정합니다. 0: 사용안함. ", "../assets/offroad/icon_road.png", 0, 100, 10));
    latLongToggles->addItem(new QLabel("Custom profile for Normal acceleration profile setting.", this));

    latLongToggles->addItem(new CValueControl("CruiseMaxVals1", "ACCEL:0km/h(160)", "속도별 가속도를 지정합니다.(x0.01m/s^2)", "../assets/offroad/icon_road.png", 1, 250, 5));
    latLongToggles->addItem(new CValueControl("CruiseMaxVals2", "ACCEL:40km/h(120)", "속도별 가속도를 지정합니다.(x0.01m/s^2)", "../assets/offroad/icon_road.png", 1, 250, 5));
    latLongToggles->addItem(new CValueControl("CruiseMaxVals3", "ACCEL:60km/h(100)", "속도별 가속도를 지정합니다.(x0.01m/s^2)", "../assets/offroad/icon_road.png", 1, 250, 5));
    latLongToggles->addItem(new CValueControl("CruiseMaxVals4", "ACCEL:80km/h(80)", "속도별 가속도를 지정합니다.(x0.01m/s^2)", "../assets/offroad/icon_road.png", 1, 250, 5));
    latLongToggles->addItem(new CValueControl("CruiseMaxVals5", "ACCEL:110km/h(70)", "속도별 가속도를 지정합니다.(x0.01m/s^2)", "../assets/offroad/icon_road.png", 1, 250, 5));
    latLongToggles->addItem(new CValueControl("CruiseMaxVals6", "ACCEL:140km/h(60)", "속도별 가속도를 지정합니다.(x0.01m/s^2)", "../assets/offroad/icon_road.png", 1, 250, 5));

    dispToggles = new ListWidget(this);
    dispToggles->addItem(new CValueControl("ShowHudMode", "DISP:Display Mode", "0:Frog,1:APilot,2:Bottom,3:Top,4:Left,5:Left-Bottom", "../assets/offroad/icon_shell.png", 0, 5, 1));
    dispToggles->addItem(new ParamControl("ShowDebugUI", "DISP:Debug Info", "", "../assets/offroad/icon_shell.png", this));
    dispToggles->addItem(new CValueControl("ShowDateTime", "DISP:Time Info", "0:None,1:Time/Date,2:Time,3:Date", "../assets/offroad/icon_shell.png", 0, 3, 1));
    dispToggles->addItem(new CValueControl("ShowSteerRotate", "DISP:Handle rotate", "0:None,1:Rotate", "../assets/offroad/icon_shell.png", 0, 1, 1));
    dispToggles->addItem(new CValueControl("ShowPathEnd", "DISP:Path End", "0:None,1:Display", "../assets/offroad/icon_shell.png", 0, 2, 1));
    dispToggles->addItem(new CValueControl("ShowAccelRpm", "DISP:Accel meter", "0:None,1:Display,1:Accel+RPM", "../assets/offroad/icon_shell.png", 0, 2, 1));
    dispToggles->addItem(new CValueControl("ShowTpms", "DISP:TPMS", "0:None,1:Display", "../assets/offroad/icon_shell.png", 0, 1, 1));
    dispToggles->addItem(new CValueControl("ShowSteerMode", "DISP:Handle Display Mode", "0:Black,1:Color,2:None", "../assets/offroad/icon_shell.png", 0, 2, 1));
    dispToggles->addItem(new CValueControl("ShowDeviceState", "DISP:Device State", "0:None,1:Display", "../assets/offroad/icon_shell.png", 0, 1, 1));
    dispToggles->addItem(new CValueControl("ShowConnInfo", "DISP:NDA connection", "0:NOne,1:Display", "../assets/offroad/icon_shell.png", 0, 1, 1));
    dispToggles->addItem(new CValueControl("ShowLaneInfo", "DISP:Lane Info", "-1:None, 0:Path, 1:Path+Lane, 2: Path+Lane+RoadEdge", "../assets/offroad/icon_shell.png", -1, 2, 1));
    dispToggles->addItem(new CValueControl("ShowBlindSpot", "DISP:BSD Info", "0:None,1:Display", "../assets/offroad/icon_shell.png", 0, 1, 1));
    dispToggles->addItem(new CValueControl("ShowGapInfo", "DISP:GAP Info", "0:None,1:Display", "../assets/offroad/icon_shell.png", -1, 1, 1));
    dispToggles->addItem(new CValueControl("ShowDmInfo", "DISP:DM Info", "0:None,1:Display,-1:Disable(Reboot)", "../assets/offroad/icon_shell.png", -1, 1, 1));
    dispToggles->addItem(new CValueControl("ShowRadarInfo", "DISP:Radar Info", "0:None,1:Display,2:RelPos,3:Stopped Car", "../assets/offroad/icon_shell.png", 0, 3, 1));
    dispToggles->addItem(new CValueControl("ShowPlotMode", "DISP:Debug plot", "", "../assets/offroad/icon_shell.png", 0, 5, 1));


    pathToggles = new ListWidget(this);
    pathToggles->addItem(new CValueControl("ShowZOffset", "DISP:Path Height adjust(170)", "(+) Down, (-) Up", "../assets/offroad/icon_shell.png", -300, 300, 10));
    pathToggles->addItem(new CValueControl("ShowPathModeCruiseOff", "DISP: Path Mode: Cruise OFFF", "0:Normal,1,2:Rec,3,4:^^,5,6:Rec,7,8:^^,9,10,11,12:Smooth^^", "../assets/offroad/icon_shell.png", 0, 15, 1));
    pathToggles->addItem(new CValueControl("ShowPathColorCruiseOff", "DISP: Path Color: Cruise OFF", "(+10:Stroke)0:Red,1:Orange,2:Yellow,3:Green,4:Blue,5:Indigo,6:Violet,7:Brown,8:White,9:Black", "../assets/offroad/icon_shell.png", 0, 19, 1));
    pathToggles->addItem(new CValueControl("ShowPathMode", "DISP:Path Mode: Laneless", "0:Normal,1,2:Rec,3,4:^^,5,6:Rec,7,8:^^,9,10,11,12:Smooth^^", "../assets/offroad/icon_shell.png", 0, 15, 1));
    pathToggles->addItem(new CValueControl("ShowPathColor", "DISP:Path Color: Laneless", "(+10:Stroke)0:Red,1:Orange,2:Yellow,3:Green,4:Blue,5:Indigo,6:Violet,7:Brown,8:White,9:Black", "../assets/offroad/icon_shell.png", 0, 19, 1));
    pathToggles->addItem(new CValueControl("ShowPathModeLane", "DISP:Path Mode: LaneMode", "0:Normal,1,2:Rec,3,4:^^,5,6:Rec,7,8:^^,9,10,11,12:Smooth^^", "../assets/offroad/icon_shell.png", 0, 15, 1));
    pathToggles->addItem(new CValueControl("ShowPathColorLane", "DISP:Path Color: LaneMode", "(+10:Stroke)0:Red,1:Orange,2:Yellow,3:Green,4:Blue,5:Indigo,6:Violet,7:Brown,8:White,9:Black", "../assets/offroad/icon_shell.png", 0, 19, 1));
    pathToggles->addItem(new CValueControl("ShowPathWidth", "DISP:Path Width ratio(100%)", "", "../assets/offroad/icon_shell.png", 10, 200, 10));

    startToggles = new ListWidget(this);
    startToggles->addItem(new ParamControl("SccConnectedBus2", "(HKG) SCC Module connected BUS2", "", "../assets/offroad/icon_warning.png", this));
    startToggles->addItem(new ParamControl("EnableRadarTracks", "(HKG) EnableRadarTracks", "Activate Radartracks at startup", "../assets/offroad/icon_warning.png", this));
    startToggles->addItem(new CValueControl("SoundVolumeAdjust", "Adjust Sound Volume(100%)", "", "../assets/offroad/icon_shell.png", 5, 100, 5));

    speedToggles = new ListWidget(this);
    speedToggles->addItem(new CValueControl("AutoCurveSpeedCtrlUse", "CURVE: Auto Control(1)", "곡선도로를 만나면 속도를 줄여줍니다. 0:사용안함,1:도로설계기준", "../assets/offroad/icon_road.png", 0, 3, 1));
    speedToggles->addItem(new CValueControl("AutoCurveSpeedFactor", "CURVE: Auto Control ratio(100%)", "커브속도조절(커브속도 조절 3일때 170)", "../assets/offroad/icon_road.png", 50, 300, 1));
    speedToggles->addItem(new CValueControl("AutoCurveSpeedFactorIn", "CURVE: Auto Control ratio In(10%)", "커브속도조절진입", "../assets/offroad/icon_road.png", 0, 300, 1));
    speedToggles->addItem(new CValueControl("AutoNaviSpeedCtrlEnd", "SpeedCameraDecelEnd(6s)", "감속완료시점을 설정합니다.값이 크면 카메라에서 멀리 감속 완료", ".. / assets / offroad / icon_road.png", 3, 20, 1));
    speedToggles->addItem(new CValueControl("AutoNaviSpeedDecelRate", "SpeedCameraDecelRatex0.01m/s^2(80)", "낮으면 멀리서부터 감속함", ".. / assets / offroad / icon_road.png", 10, 200, 10));
    speedToggles->addItem(new CValueControl("AutoNaviSpeedSafetyFactor", "SpeedCameraSafetyFactor(105%)", "", ".. / assets / offroad / icon_road.png", 80, 120, 1));
    speedToggles->addItem(new CValueControl("AutoNaviSpeedBumpTime", "SpeedBumpTimeDistance(1s)", "", ".. / assets / offroad / icon_road.png", 1, 50, 1));
    speedToggles->addItem(new CValueControl("AutoNaviSpeedBumpSpeed", "SpeedBumpSpeed(35Km/h)", "", ".. / assets / offroad / icon_road.png", 10, 100, 5));

    toggles_layout->addWidget(cruiseToggles);
    toggles_layout->addWidget(latLongToggles);
    toggles_layout->addWidget(dispToggles);
    toggles_layout->addWidget(pathToggles);
    toggles_layout->addWidget(startToggles);
    toggles_layout->addWidget(speedToggles);
    ScrollView* toggles_view = new ScrollView(toggles, this);
    carrotLayout->addWidget(toggles_view, 1);

    homeScreen->setLayout(carrotLayout);
    main_layout->addWidget(homeScreen);
    main_layout->setCurrentWidget(homeScreen);

    togglesCarrot(4);

    setStyleSheet(R"(
    #start_btn { height: 120px; border-radius: 15px; background-color: #393939; }
    #start_btn:pressed { background-color: #4a4a4a; }
    #cruise_btn { height: 120px; border-radius: 15px; background-color: #393939; }
    #cruise_btn:pressed { background-color: #4a4a4a; }
    #speed_btn { height: 120px; border-radius: 15px; background-color: #393939; }
    #speed_btn:pressed { background-color: #4a4a4a; }
    #latLong_btn { height: 120px; border-radius: 15px; background-color: #393939; }
    #latLong_btn:pressed { background-color: #4a4a4a; }
    #disp_btn { height: 120px; border-radius: 15px; background-color: #393939; }
    #disp_btn:pressed { background-color: #4a4a4a; }
    #path_btn { height: 120px; border-radius: 15px; background-color: #393939; }
    #path_btn:pressed { background-color: #4a4a4a; }
  )");

}
void CarrotPanel::togglesCarrot(int widgetIndex) {
    cruiseToggles->setVisible(widgetIndex==0);
    latLongToggles->setVisible(widgetIndex == 1);
    dispToggles->setVisible(widgetIndex == 2);
    pathToggles->setVisible(widgetIndex == 3);
    startToggles->setVisible(widgetIndex == 4);
    speedToggles->setVisible(widgetIndex == 5);
}

// ajouatom
CValueControl::CValueControl(const QString& params, const QString& title, const QString& desc, const QString& icon, int min, int max, int unit/*=1*/) : AbstractControl(title, desc, icon)
{

    m_params = params;
    m_min = min;
    m_max = max;
    m_unit = unit;

    label.setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    label.setStyleSheet("color: #e0e879");
    hlayout->addWidget(&label);

    btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
    btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
    btnminus.setFixedSize(150, 100);
    btnplus.setFixedSize(150, 100);
    //btnminus.setAutoRepeat(true);
    //btnminus.setAutoRepeatInterval(150);
    //btnplus.setAutoRepeat(true);
    //btnplus.setAutoRepeatInterval(150);
    hlayout->addWidget(&btnminus);
    hlayout->addWidget(&btnplus);

    QObject::connect(&btnminus, &QPushButton::released, [=]() {
        auto str = QString::fromStdString(Params().get(m_params.toStdString()));
        int value = str.toInt();
        value = value - m_unit;
        if (value < m_min) {
            value = m_min;
        }
        else {
        }

        Params().putInt(m_params.toStdString(), value);
        refresh();
    });

    QObject::connect(&btnplus, &QPushButton::released, [=]() {
        auto str = QString::fromStdString(Params().get(m_params.toStdString()));
        int value = str.toInt();
        value = value + m_unit;
        if (value > m_max) {
            value = m_max;
        }
        else {
        }

        Params().putInt(m_params.toStdString(), value);
        refresh();
    });
    refresh();
}

void CValueControl::refresh()
{
    label.setText(QString::fromStdString(Params().get(m_params.toStdString())));
    btnminus.setText("－");
    btnplus.setText("＋");
}
