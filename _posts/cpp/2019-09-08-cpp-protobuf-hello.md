---
layout: post
title: Google protobuf cpp hello world
categories: cpp
tags: [protobuf]
image:
public: true
description: Google protobuf hello world in cpp
---

```cpp
#include <iostream>
#include "messages.pb.h"

using namespace std;
int main()
{
    Messages::Msg m;
    m.set_data("hello proto");
    auto buf = m.SerializeAsString();
    Messages::Msg new_m;
    new_m.ParseFromString(buf);
    cout << new_m.data() << endl;
    return 0;
}
```

## meson
```python
protoc = find_program('protoc')
dep = dependency('protobuf')

gen = generator(protoc, \
  output    : ['@BASENAME@.pb.cc', '@BASENAME@.pb.h'],
  arguments : ['--proto_path=@CURRENT_SOURCE_DIR@/proto/', 
  '--cpp_out=@BUILD_DIR@', '@INPUT@'])
# @BUILD_DIR@
generated = gen.process('proto/messages.proto')
e = executable('prog', 'main.cpp', generated,
dependencies : dep)
```


# qt5
```
project('cat', 'cpp')
subdir('src/protobuf_demo')
# src = ['src/main.cpp', 'src/MainWindow.cpp']
# qt5 = import('qt5')
# includes = include_directories('src')

# qt5_deps = dependency('qt5', modules: ['Core', 
#     'Gui', 
#     'Widgets'])

# moc_files = qt5.preprocess(moc_headers : 'src/MainWindow.hpp',
#                            moc_extra_arguments: ['-DMAKES_MY_MOC_HEADER_COMPILE'],
#                            include_directories: includes,
#                            dependencies: qt5_deps)

# executable('cat', src, 
#     moc_files, 
#     dependencies: [qt5_deps],
#     include_directories: [includes])
```

## main
```cpp
#include <QApplication>
#include <MainWindow.hpp>
// #include <QPushButton>

// int main(int argc, char* argv[]) {
//  QApplication app(argc, argv);
//  QPushButton btn("hello");
//  btn.setText("world");
//  btn.setCursor(Qt::PointingHandCursor);
//  btn.show();
//  return app.exec();
// }
    int main(int argc, char * argv[])
    {
      QApplication a(argc, argv);
      MainWidget w;
      w.show();
      return a.exec();
    }
```

## win
```cpp
#include "MainWindow.hpp"

#include <QWidget>
#include <QGridLayout>
// #include <QStatusBar>
class Controller{
    private:
        MainWidget *widget_;

    public:
        Controller(MainWidget *widget)
        {
            widget_ = widget;
        }
};

MainWidget::MainWidget(QWidget *parent)
    : QWidget(parent)
{
    // == WINDOW SETTINGS ==
    setWindowTitle("Basic Application");
    Controller ctl(this);

    setMinimumSize(800, 450);
    button_ = new QPushButton(tr("Push Me!"));
    // Connect button signal to appropriate slot
    connect(button_,
        SIGNAL (released()),
        this,
        SLOT (onButtonReleased()));

    textBrowser_ = new QTextBrowser();

    QGridLayout *mainLayout = new QGridLayout;
    mainLayout->addWidget(button_, 0, 0);
    mainLayout->addWidget(textBrowser_, 1, 0);
    setLayout(mainLayout);
}

// Handler for button click
void MainWidget::onButtonReleased()
{
    // clear the text in the textBrowser
    textBrowser_->clear();
    textBrowser_->append(tr("Running command:"));
    
}
```

#  win header
```cpp
#pragma once

#include <QWidget>
#include <QPushButton>
#include <QTextBrowser>

class MainWidget : public QWidget
{
    Q_OBJECT

public:
    MainWidget(QWidget *parent = nullptr);


private slots:
    void onButtonReleased(); // Handler for button presses
    
private:
   QPushButton* button_;
   QTextBrowser* textBrowser_;
};
```


[cp](https://www.codeproject.com/Articles/mehreentahir16#Article)