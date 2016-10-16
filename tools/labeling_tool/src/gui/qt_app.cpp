#include "qt_app.hpp"

#include <QApplication>
#include "main_window.h"

namespace gui
{
int App::run(int argc, char* argv[])
{
    QApplication app(argc, argv);

    MainWindow window;
    window.show();

    return app.exec();
}

}
