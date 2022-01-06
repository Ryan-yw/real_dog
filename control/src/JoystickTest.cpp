#include "./control/include/JoystickTest.h"
#include "ui_JoystickTest.h"
#include <QTimer>

std::atomic<double> gamepad[2] = {0};

JoystickTestWindow::JoystickTestWindow(GameController& gamepad, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::JoystickTestWindow),
    _gamepad(gamepad)
{
    ui->setupUi(this);

  QTimer *timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(update()));
  timer->start(1000 / 30);
}

JoystickTestWindow::~JoystickTestWindow()
{
    delete ui;
}


void JoystickTestWindow::update() {
  _gamepad.updateGamepadCommand(_command);
  char buffer[256];

  //gamepad[0] = gamepad[0] * (1 - std::abs(command_.rightStickAnalog[0])) +  command_.leftStickAnalog[0] * std::abs(command_.rightStickAnalog[0]);
  gamepad[0] = gamepad[0] * (1 - 0.1) +  _command.rightStickAnalog[1] * 0.1; // 0->left right   1->forward back
  gamepad[1] = gamepad[1] * (1 - 0.1) +  _command.rightStickAnalog[0] * 0.1;
  //gamepad[1] = gamepad[1] * (1 - 0.2) +  1 * 0.2;

  sprintf(buffer, "Left X: %4.2f\n", _command.leftStickAnalog[0]);
  ui->leftXLabel->setText(buffer);

  sprintf(buffer, "Left Y: %4.2f\n", _command.leftStickAnalog[1]);
  ui->leftYLabel->setText(buffer);

  sprintf(buffer, "Right X: %4.2f\n", _command.rightStickAnalog[0]);
  ui->rightXLabel->setText(buffer);

  sprintf(buffer, "Right Y: %4.2f\n", _command.rightStickAnalog[1]);
  ui->rightYLabel->setText(buffer);
}
