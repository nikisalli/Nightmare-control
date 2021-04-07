from PyQt5 import QtCore, QtGui, QtWidgets


class RadialBar(QtWidgets.QWidget):
    class DialType():
        FullDial = 0
        MinToMax = 1
        NoDial = 2

    sizeChanged = QtCore.pyqtSignal()
    startAngleChanged = QtCore.pyqtSignal()
    spanAngleChanged = QtCore.pyqtSignal()
    minValueChanged = QtCore.pyqtSignal()
    maxValueChanged = QtCore.pyqtSignal()
    valueChanged = QtCore.pyqtSignal()
    dialWidthChanged = QtCore.pyqtSignal()
    backgroundColorChanged = QtCore.pyqtSignal()
    foregroundColorChanged = QtCore.pyqtSignal()
    progressColorChanged = QtCore.pyqtSignal()
    textColorChanged = QtCore.pyqtSignal()
    suffixTextChanged = QtCore.pyqtSignal()
    showTextChanged = QtCore.pyqtSignal()
    penStyleChanged = QtCore.pyqtSignal()
    dialTypeChanged = QtCore.pyqtSignal()
    textFontChanged = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super(RadialBar, self).__init__(parent)

        self.resize(150, 150)
        # self.setSmooth(True)
        # self.setAntialiasing(True)

        self._Size = 200
        self._StartAngle = 40
        self._SpanAngle = 280
        self._MinValue = 0
        self._MaxValue = 9
        self._Value = 5
        self._DialWidth = 12
        self._BackgroundColor = QtCore.Qt.transparent
        self._DialColor = QtGui.QColor(80, 80, 80)
        self._ProgressColor = QtGui.QColor(42, 100, 80)
        self._TextColor = QtGui.QColor(0, 0, 0)
        self._SuffixText = "V"
        self._ShowText = True
        self._PenStyle = QtCore.Qt.RoundCap
        self._DialType = RadialBar.DialType.MinToMax
        self._TextFont = QtGui.QFont()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        # painter.save()
        r = min(self.width, self.height)
        rect = QtCore.QRectF(0, 0, r, r)  # self.boundingRect()
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        pen = painter.pen()
        pen.setCapStyle(self._PenStyle)

        startAngle = -90 - self._StartAngle
        if RadialBar.DialType.FullDial != self._DialType:
            spanAngle = 0 - self._SpanAngle
        else:
            spanAngle = -360

        # Draw outer dial
        painter.save()
        pen.setWidth(self._DialWidth)
        pen.setColor(self._DialColor)
        painter.setPen(pen)
        offset = self._DialWidth / 2
        if self._DialType == RadialBar.DialType.MinToMax:
            painter.drawArc(rect.adjusted(offset, offset, -offset, -offset), startAngle * 16, spanAngle * 16)
        elif self._DialType == RadialBar.DialType.FullDial:
            painter.drawArc(rect.adjusted(offset, offset, -offset, -offset), -90 * 16, -360 * 16)
        else:
            pass
            # do not draw dial

        painter.restore()

        # Draw background
        painter.save()
        painter.setBrush(self._BackgroundColor)
        painter.setPen(self._BackgroundColor)
        inner = offset * 2
        painter.drawEllipse(rect.adjusted(inner, inner, -inner, -inner))
        painter.restore()

        # Draw progress text with suffix
        painter.save()
        painter.setFont(self._TextFont)
        pen.setColor(self._TextColor)
        painter.setPen(pen)
        if self._ShowText:
            painter.drawText(rect.adjusted(offset, offset, -offset, -offset), QtCore.Qt.AlignCenter, str(self._Value) + self._SuffixText)
        else:
            painter.drawText(rect.adjusted(offset, offset, -offset, -offset), QtCore.Qt.AlignCenter, self._SuffixText)
        painter.restore()

        # Draw progress bar
        painter.save()
        pen.setWidth(self._DialWidth)
        pen.setColor(self._ProgressColor)
        valueAngle = float(float(self._Value - self._MinValue)/float(self._MaxValue - self._MinValue)) * float(spanAngle)  # Map value
        painter.setPen(pen)
        painter.drawArc(rect.adjusted(offset, offset, -offset, -offset), startAngle * 16, valueAngle * 16)
        painter.restore()

    @QtCore.pyqtProperty(str, notify=sizeChanged)
    def size(self):
        return self._Size

    @size.setter
    def size(self, size):
        if self._Size == size:
            return
        self._Size = size
        self.sizeChanged.emit()
        self.update()

    @QtCore.pyqtProperty(int, notify=startAngleChanged)
    def startAngle(self):
        return self._StartAngle

    @startAngle.setter
    def startAngle(self, angle):
        if self._StartAngle == angle:
            return
        self._StartAngle = angle
        self.startAngleChanged.emit()
        self.update()

    @QtCore.pyqtProperty(int, notify=spanAngleChanged)
    def spanAngle(self):
        return self._SpanAngle

    @spanAngle.setter
    def spanAngle(self, angle):
        if self._SpanAngle == angle:
            return
        self._SpanAngle = angle
        self.spanAngleChanged.emit()
        self.update()

    @QtCore.pyqtProperty(int, notify=minValueChanged)
    def minValue(self):
        return self._MinValue

    @minValue.setter
    def minValue(self, value):
        if self._MinValue == value:
            return
        self._MinValue = value
        self.minValueChanged.emit()
        self.update()

    @QtCore.pyqtProperty(int, notify=maxValueChanged)
    def maxValue(self):
        return self._MaxValue

    @maxValue.setter
    def maxValue(self, value):
        if self._MaxValue == value:
            return
        self._MaxValue = value
        self.maxValueChanged.emit()
        self.update()

    @QtCore.pyqtProperty(float, notify=valueChanged)
    def value(self):
        return self._Value

    @value.setter
    def value(self, value):
        if self._Value == value:
            return
        self._Value = value
        self.valueChanged.emit()
        self.update()

    @QtCore.pyqtProperty(int, notify=dialWidthChanged)
    def dialWidth(self):
        return self._DialWidth

    @dialWidth.setter
    def dialWidth(self, width):
        if self._DialWidth == width:
            return
        self._DialWidth = width
        self.dialWidthChanged.emit()
        self.update()

    @QtCore.pyqtProperty(QtGui.QColor, notify=backgroundColorChanged)
    def backgroundColor(self):
        return self._BackgroundColor

    @backgroundColor.setter
    def backgroundColor(self, color):
        if self._BackgroundColor == color:
            return
        self._BackgroundColor = color
        self.backgroundColorChanged.emit()
        self.update()

    @QtCore.pyqtProperty(QtGui.QColor, notify=foregroundColorChanged)
    def foregroundColor(self):
        return self._ForegrounColor

    @foregroundColor.setter
    def foregroundColor(self, color):
        if self._DialColor == color:
            return
        self._DialColor = color
        self.foregroundColorChanged.emit()
        self.update()

    @QtCore.pyqtProperty(QtGui.QColor, notify=progressColorChanged)
    def progressColor(self):
        return self._ProgressColor

    @progressColor.setter
    def progressColor(self, color):
        if self._ProgressColor == color:
            return
        self._ProgressColor = color
        self.progressColorChanged.emit()
        self.update()

    @QtCore.pyqtProperty(QtGui.QColor, notify=textColorChanged)
    def textColor(self):
        return self._TextColor

    @textColor.setter
    def textColor(self, color):
        if self._TextColor == color:
            return
        self._TextColor = color
        self.textColorChanged.emit()
        self.update()

    @QtCore.pyqtProperty(str, notify=suffixTextChanged)
    def suffixText(self):
        return self._SuffixText

    @suffixText.setter
    def suffixText(self, text):
        if self._SuffixText == text:
            return
        self._SuffixText = text
        self.suffixTextChanged.emit()
        self.update()

    @QtCore.pyqtProperty(str, notify=showTextChanged)
    def showText(self):
        return self._ShowText

    @showText.setter
    def showText(self, show):
        if self._ShowText == show:
            return
        self._ShowText = show
        self.update()

    @QtCore.pyqtProperty(QtCore.Qt.PenCapStyle, notify=penStyleChanged)
    def penStyle(self):
        return self._PenStyle

    @penStyle.setter
    def penStyle(self, style):
        if self._PenStyle == style:
            return
        self._PenStyle = style
        self.penStyleChanged.emit()
        self.update()

    @QtCore.pyqtProperty(int, notify=dialTypeChanged)
    def dialType(self):
        return self._DialType

    @dialType.setter
    def dialType(self, type):
        if self._DialType == type:
            return
        self._DialType = type
        self.dialTypeChanged.emit()
        self.update()

    @QtCore.pyqtProperty(QtGui.QFont, notify=textFontChanged)
    def textFont(self):
        return self._TextFont

    @textFont.setter
    def textFont(self, font):
        if self._TextFont == font:
            return
        self._TextFont = font
        self.textFontChanged.emit()
        self.update()


if __name__ == '__main__':
    import sys
    app = QtWidgets.QApplication(sys.argv)
    w = RadialBar()
    w.backgroundColor = QtGui.QColor("#1dc58f")
    w.foregroundColor = QtGui.QColor("#191a2f")
    w.dialWidth = 10
    w.spanAngle = 70
    w.textColor = QtGui.QColor("#FFFFFF")
    w.penStyle = QtCore.Qt.RoundCap
    w.dialType = RadialBar.DialType.FullDial
    w.suffixText = "%"
    timeline = QtCore.QTimeLine(10000, w)
    timeline.setFrameRange(0, 100)
    timeline.frameChanged.connect(lambda val: setattr(w, "value", val))
    timeline.start()
    w.show()
    sys.exit(app.exec_())
