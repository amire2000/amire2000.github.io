---
layout: post
title: Qt using PySide2 to implement MVC
categories: python  
tags: [mvc, qt]
public: true
description: Using PySide to implement MVC pattern
image: mvc.png
---

# Install Qt creator

![](/images/2019-08-24-12-56-57.png)

![](/images/2019-08-24-13-43-25.png)

# Tools

## convert xml ui to python
```
pyuic5 main_view.ui -o main_view_ui.py
```

- pyuic is part of `pyqt5-dev-tools`
  
```
sudo apt install pyqt5-dev-tools
```

## main (mvc_app.py)
```python
import sys
from model.model import Model
from controllers.main_ctrl import MainController
from views.main_view import MainView

from PySide2.QtWidgets import QApplication

class App(QApplication):
    def __init__(self, sys_args):
        super(App, self).__init__(sys_args)
        self.model = Model()
        self.main_view = MainView()
        self.main_controller = MainController(self.model, self.main_view)
        self.main_view.show()
        
        

if __name__ == "__main__":
    app = App(sys.argv)
    sys.exit(app.exec_())

```
## view (xml)
```xml
<?xml version="1.0" encoding="UTF-8"?>
<ui version="4.0">
 <class>MainWindow</class>
 <widget class="QMainWindow" name="MainWindow">
  <property name="geometry">
   <rect>
    <x>0</x>
    <y>0</y>
    <width>93</width>
    <height>86</height>
   </rect>
  </property>
  <widget class="QWidget" name="centralwidget">
   <layout class="QVBoxLayout">
    <item>
     <widget class="QSpinBox" name="spinBox_amount"/>
    </item>
    <item>
     <widget class="QLabel" name="label_even_odd"/>
    </item>
    <item>
     <widget class="QPushButton" name="pushButton_reset">
      <property name="enabled">
       <bool>false</bool>
      </property>
     </widget>
    </item>
   </layout>
  </widget>
 </widget>
 <resources/>
 <connections/>
</ui>
```

## view (main_view.py)
```python
from PySide2.QtWidgets import QMainWindow
from PySide2.QtCore import Slot
from views.main_view_ui import Ui_MainWindow


class MainView(QMainWindow):
    def __init__(self):
        super().__init__()

        
        self._ui = Ui_MainWindow()
        self._ui.setupUi(self)
        
        # # connect widgets to controller
        

    @property
    def ui(self):
        return self._ui
        # # set a default value
```

## controller (main_ctrl.py)
```python
from PySide2.QtCore import QObject, Slot


class MainController(QObject):
    def __init__(self, model, view):
        super().__init__()
        self._view = view
        self._model = model 
        # listen for model event signals
        self._model.amount_changed += self.on_amount_changed
        self._model.even_odd_changed += self.on_even_odd_changed
        self._model.enable_reset_changed += self.on_enable_reset_changed

        self._view.ui.spinBox_amount.valueChanged.connect(self.change_amount)

        self._view.ui.pushButton_reset.clicked.connect(lambda: self.change_amount(0))

        self.change_amount(42)

    def change_amount(self, value):
        self._model.amount = value

        # calculate even or odd
        self._model.even_odd = 'odd' if value % 2 else 'even'

        # calculate button enabled state
        self._model.enable_reset = True if value else False

    def on_amount_changed(self, value):
        self._view.ui.spinBox_amount.setValue(value)

    def on_even_odd_changed(self, value):
        self._view.ui.label_even_odd.setText(value)

    def on_enable_reset_changed(self, value):
        self._view.ui.pushButton_reset.setEnabled(value)
```

## model (model.py)
```python
class EventHook(object):

    def __init__(self):
        self.__handlers = []

    def __iadd__(self, handler):
        self.__handlers.append(handler)
        return self

    def __isub__(self, handler):
        self.__handlers.remove(handler)
        return self

    def fire(self, *args, **keywargs):
        for handler in self.__handlers:
            handler(*args, **keywargs)


class Model(EventHook):
    amount_changed = EventHook()
    even_odd_changed = EventHook()
    enable_reset_changed = EventHook()

    @property
    def amount(self):
        return self._amount

    @amount.setter
    def amount(self, value):
        self._amount = value
        self.amount_changed.fire(value)

    @property
    def even_odd(self):
        return self._even_odd

    @even_odd.setter
    def even_odd(self, value):
        self._even_odd = value
        self.even_odd_changed.fire(value)

    @property
    def enable_reset(self):
        return self._enable_reset

    @enable_reset.setter
    def enable_reset(self, value):
        self._enable_reset = value
        self.enable_reset_changed.fire(value)

    def __init__(self):
        super().__init__()

        self._amount = 0
        self._even_odd = ''
        self._enable_reset = False
```


# Reference
- [mvc](https://stackoverflow.com/questions/26698628/mvc-design-with-qt-designer-and-pyqt-pyside)
- [qt creator](https://stackoverflow.com/questions/24100602/developing-python-applications-in-qt-creator)
- [tkmvvm](https://github.com/Joklost/tkmvvm)
- [simple gui designer](https://github.com/alejandroautalan/pygubu)