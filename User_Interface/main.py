#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Sampler es una aplicación ideada para mejorar la eficacia del proceso de muestreo y lectura de sensores de un banco de
# pruebas para helices con motores de RC. Permite controlar periodos de lectura de 3 sensores tipo celda de carga y un sensor
# tacómetro. Además, habilita el control del Throttle del motor.

# Las lecturas se pueden realizar manualmente en un modo de barrido o automáticamente especificando el número de
# etapas de throttle dentro de un rango definido y el periodo de duración de cada etapa.


import serial, struct
import serial.tools.list_ports as serialP
from PyQt5 import QtCore
from PyQt5.QtWidgets import QMainWindow, QApplication, QWidget, QLabel, QVBoxLayout, QCheckBox
from PyQt5.uic import loadUi
import sys
import matplotlib
import numpy as np
matplotlib.use('Qt5Agg')

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT
from matplotlib.figure import Figure

class Main(QMainWindow):                    # La clase principal de la aplicación, donde todas las variables, métodos y objetos utilizados
    def __init__(self):                     # por esta son declarados. La interfaz gráfica de Sampler se desarrolló en Qt y parte de esta
        super(Main, self).__init__()        # se creó por medio de QtDesigner, mientras que las gráficas se programaron dentro de este
        loadUi("sampler.ui", self)          # script. La función "loadUI("sampler.ui", self)" del módulo PyQt5.uic carga la parte de la 
        self.plot1 = None                   # interfaz hecha en Designer al script para que sus Widgets y atributos puedan ser manipulados
        self.recordT = []                   # directamente desde aqui. Los nombres de las Widgets exportadas son declarados desde Designer
        self.recordM = []
        self.recordR = []
        
        self.actionExport.triggered.connect(self.export)
        self.actionSweep_2.triggered.connect(self.modeSweep)
        self.actionPeriod.triggered.connect(self.modePeriod)
        
        self.actionCheckCom.triggered.connect(self.comCheck)
        self.actionReset.triggered.connect(self.resetText)
        self.actionPlot.triggered.connect(self.showPlot)
        
        self.actionRunSweep.triggered.connect(self.runSampleSweep)
        self.actionStopSweep.triggered.connect(self.stopSampleSweep)
        self.actionRunPeriod.triggered.connect(self.runSamplePeriod)
        self.rpmSlider.valueChanged.connect(self.updateRPM)
        
        self.actionReset.setEnabled(False)
        self.actionPlot.setEnabled(False)
        self.actionRunSweep.setEnabled(False)
        self.actionStopSweep.setEnabled(False)
        self.actionRunPeriod.setEnabled(False)
        self.rpmSlider.setEnabled(False)
        self.samplePeriodText.hide()
        self.sampleNumberText.hide()
        self.PeriodLabel.hide()
        self.StepLabel.hide()
        
        
        self.statusInfo = QLabel()
        self.statusInfo.setText("No Connection.    No mode")
        self.statusbar.addPermanentWidget(self.statusInfo)
        
        self.timerSweep = QtCore.QTimer()
        self.autoPeriodCountDownTimer = QtCore.QTimer()
        self.timerAutoPeriod = QtCore.QTimer()
        
        self.sweepSamplingInterval = 10
        self.periodSamplingInterval = 10
        
        self.timerSweep.setInterval(self.sweepSamplingInterval)
        self.autoPeriodCountDownTimer.setInterval(1000)
        self.timerAutoPeriod.setInterval(self.periodSamplingInterval)
        
        self.timerSweep.timeout.connect(self.updateSampleSweep)
        self.autoPeriodCountDownTimer.timeout.connect(self.updateCountdown)
        self.timerAutoPeriod.timeout.connect(self.updateSamplePeriod)
        
# De la linea 26 a la 73 se define el constructor/inicializador de la clase. Una vez cargado el archivo .ui elaborado en QtDesigner, todas las widgets
# colocadas a travez de Designer se vuelven manipulables en el script. A continuación un desglose de los cambios de atributos de Widgets
# presentes en el constructor:
#   
#       Nombre de Widget            Cambio 
             
#       actionExport                Conectada al método self.export
#       actionSweep_2               Conectada al método self.modeSweep
#       actionPeriod                Conectada al método self.modePeriod
#       actionCheckCom              ""  self.comCheck
#       actionReset                 ""  self.resetText
#       actionPlot                  ""  self.showPlot
#       actionRunSweep              ""  self.runSampleSweep
#       actionStopSweep             ""  self.stopSampleSweep
#       actionRunPeriod             ""  self.runSamplePeriod
#       rpmSlider                   ""  self.updateRPM  
                   
#       actionReset                 Deshabilitado al inicio
#       actionPlot                  Deshabilitado al inicio
#       actionRunSweep              Deshabilitado al inicio
#       actionStopSweep             Deshabilitado al inicio
#       actionRunPeriod             Deshabilitado al inicio
#       rpmSlider                   Deshabilitado al inicio
#       samplePeriodText            Ocultado al inicio
#       sampleNumberText            Ocultado al inicio
#       PeriodLabel                 Ocultado al inicio
#       StepLabel                   Ocultado al inicio   

#       statusInfo                  Creada como widget de tipo QLabel
#       statusInfo                  Contenido de texto modificado
#       statusbar                   Contenido modificado para incluir widget statusInfo como label

#       timerSweep                  Creado como widget de tipo QTimer

#       timerSweep                  Periodo temporal modificado

#       timerSweep                  Conectada al método self.updateSample  

# Las widgets conectadas a un método llamaran a este cuando el usuario interactue con sus acciones relacionadas, en lo que se conoce como
# evento. Por ejemplo, un evento consistente en que el usuario seleccione del menu superior el boton asociado a la widget "actionCheckCom" tendra
# como resultado la ejecución del método self.comCheck

# Las widgets deshabilitadas u ocultas al inicio son aquellas que no estan diseñadas para ser accedidas de inmediato por el usuario, ya que
# requieren de que se cumplan condiciones previas. En este caso, varios widgets requieren que antes este conectado un microcontrolador válido
# mientras que otras necesitan tambien que el usuario haya seleccionado un modo de muestreo.

# Las widgets de tipo QTimer son contadores cuya frecuencia de operación depende de las características del procesador del computador.
# Es posible establecer el límite de cuenta en formato de milisegundos para crear un temporizador. Cuando el contador se encuentra en 
# overflow (alcanza su lìmite establecido de cuenta) ejecuta el método que haya sido conectado a este para la condición "timeout".
        
# =========================================================================================================================================== #
# A continuaciòn, las variables pùblicas de la clase que necesitan ser manejadas por más de un método y una breve explicaciòn de
# sus funciones:
    
    # ==== Variables del modo de muestreo por etapas ==== #
    period = 0                      # Guardar el periodo de tiempo en milisegundos que demora cada etapa de Throttle en
                                    # concluir. Especificado por el usuario a través de la interfáz
                                    
    decreasingPeriod = 0            # Actuar como contador descendente desde 'period' hasta 0 para auxiliar a 
                                    # timerAutoPeriod a cambiar de Throttle en el tiempo adecuado. Decrementa en un valor de
                                    # 'periodSamplingInterval' cada vez que timerAutoPeriod llega a un timeout
                                    
    steps = 0                       # Guardar la cantidad de etapas de Throttle por las que Sampler debe pasar.
                                    # Especificado por el usuario a través de la interfáz
                                    
    powerSteps = 0                  # Posteriormente se convierte en un vector de tamaño 'steps' encargado de guardar las
                                    # configuraciones de Throttle correspondientes a cada etapa. Calculado por Sampler a partir de
                                    # 'steps' y el rango de Throttle especificado por el usuario
                                    
    stepIndex = 0                   # Indicar la etapa actual de Throttle durante el muestreo por etapas
    
    decreasingSteps = 0             # Actuar como contador descendente desde 'steps' hasta 0 para auxiliar a
                                    # timerAutoPeriod a automatizar el cambio de etapas y asegurar que se detenga correctamente
                                    
                                    
    # ==== Variables de la cuenta regresiva del modo muestreo por etapas ==== #
    defaultCountdown = 5
    countdown = 5
                                    
    
    # ==== Variables tipo Flags ==== #
    readStatus = 0                  # Flag encargada de comunicar si Sampler tiene activa una sesión de muestreo
    
    pauseStatus = 0                 # Flag encargada de comunicar si la sesión de muestreo activa esta pausada
    
    
    # ==== Variables adicionales ==== #
    mode = None                     # Indicar el módo de muestreo en el que actualmente se encuentra Sampler
    
    dataSets = 0                   # Indicar la cantidad de juegos de muestras que Sampler ha guardado durante el tiempo
                                    # que ha estado activo. Equivalente a la cantidad de sesiones de muestreo realizadas
                                    
    timeAxisLimit = 5000            # Guardar el valor por defecto del tamaño del eje temporal en las gráficas de las lecturas
    
    thrustAxisLimit = 3             # Guardar el valor por defecto del tamaño del eje de empuje en su respectiva gráfica
    
    torqueAxisLimit = 1             # Guardar el valor por defecto del tamaño del eje de torque en su respectiva gráfica
    
    speedAxisLimit = 10000          # Guardar el valor por defecto del tamaño del eje de velocidad angular en su respectiva gráfica
    
    xMaxT = []                      # Variables que posteriormente se convertiran en vectores encargados de recordar lecturas maximas
    yMaxT = []                      # de cada parámetro para que Sampler pueda actualizar en tiempo real los límites máximos y
    xMaxM = []                      # mínimos de cada gráfica
    yMaxM = []
    xMaxR = []
    yMaxR = []
    
    
    def export(self):                                   # Método en desarrollo
        print("Export clicked")
        
        
        
    def modeSweep(self):                                # Método llamado cuando el usuario selecciona la opción 'Sweep' del menú superior
                                                        # Ejecuta las siguientes acciones al ser llamado:
        self.mode = "Manual"                            # -> Declara el modo de lectura como 'Manual', Sweep, Lectura de Barrido
        Arduino.write(bytes("X", 'utf-8'))              # -> Ordena al microcontrolador a cambiar a modo de lectura
        
        self.samplePeriodText.hide()                    # -> Oculta o deshabilita los widgets asociados al modo Lectura por Etapas
        self.sampleNumberText.hide()
        self.PeriodLabel.hide()
        self.StepLabel.hide()
        self.actionRunPeriod.setEnabled(False)
    
        self.actionRunSweep.setEnabled(True)            # -> Muestra o habilita los widgets asociados al modo Lectura de Barrido
        self.rpmSlider.setEnabled(True)
        self.actionStopSweep.setEnabled(True)
        self.actionReset.setEnabled(True)
        self.actionPlot.setEnabled(True)
        
        if self.plot1 is not None:                      # -> En caso de que exista una instancia de gráficas activa, deshabilitar la
            self.plot1.overlayData.setEnabled(False)    # superposición de datos (función en desarrollo)
            self.plot1.overlayData.setDown(False)
            
        self.comCheck()                                 # -> Comprobar la conexión segura con el microcontrolador

        

    def modePeriod(self):                                   # Método llamado cuando el usuario selecciona la opción 'Period' del menú superior
         if (self.comCheck() == 1):                         # Ejecuta las siguientes acciones al ser llamado:
             self.mode = "Auto Period"                      # -> Declara el modo de lectura como 'Auto Period', Lectura por Etapas
             Arduino.write(bytes("X", 'utf-8'))             # -> Ordena al microcontrolador a cambiar a modo de lectura
             
             self.samplePeriodText.show()                   # -> Oculta o deshabilita los widgets asociados al modo Lectura de Barrido
             self.sampleNumberText.show()
             self.PeriodLabel.show()
             self.StepLabel.show()
             self.actionRunSweep.setEnabled(False)
             self.actionStopSweep.setEnabled(False)
             
             self.actionReset.setEnabled(True)              # -> Muestra o habilita los widgets asociados al modo Lectura por Etapas
             self.actionPlot.setEnabled(True)
             self.actionRunPeriod.setEnabled(True)
             self.rpmSlider.setEnabled(True)

             if self.plot1 is not None:                     # -> En caso de que exista una instancia de gráficas activa, deshabilitar la
                 self.plot1.overlayData.setEnabled(False)   # superposición de datos (función en desarrollo)
                 self.plot1.overlayData.setDown(False)

             self.comCheck()                                # -> Comprobar la conexión segura con el microcontrolador

            

    def resetText(self):                            # Método llamado cuando el usuario presiona el botón 'Reset' de la barra de herramientas
        self.textEdit.clear()                       # Ejecuta las siguientes acciones al ser llamado:
                                                    # -> Limpiar el contenido de la consola
        if (self.plot1 is not None):                # -> En caso de que exista una instancia de gráficas activa, llamar a 'self.resetData()'
            self.resetData()
        if (self.mode == 1):                        # -> En caso de que exista una sesión de muestreo activa, ordenar al microcontrolador a reiniciar           # 10/04/24 Removido condicional IF anidado para cuando el programa se encuentre pausado o no
            Arduino.write(bytes("t", 'utf-8'))      # las marcas temporales y limpiar el buffer de datos para exportar del puerto COM
            Arduino.reset_input_buffer()
                 

                
    def resetData(self):                            # Método llamado por 'self.resetText()' cuando existe una instancia de gráficas activa
                                                    # Ejecuta las siguientes acciones al ser llamado:
        del self.recordT                            # -> Elimina de la memoria las variables encargadas de guardar los juegos de lecturas
        del self.recordM                            
        del self.recordR
        self.recordT = []                           # -> Declara nuevamente las variables encargadas de guardar los juegos de lecturas
        self.recordM = []
        self.recordR = []
        self.dataSets = 0                           # -> Devuelve el contador de juegos de lecturas a su estado inicial
        self.recordT.append(recordedData())         # -> Convierte a las variables 'recordX' a vectores de objetos 'recordedData'
        self.recordM.append(recordedData())
        self.recordR.append(recordedData())
        self.xMaxT = []                             # -> Devuelve las variables encargadas de guardar los máximos a sus estados iniciales
        self.yMaxT = []
        self.xMaxM = []
        self.yMaxM = []
        self.xMaxR = []
        self.yMaxR = []
        if self.plot1 is not None:                      # -> En caso de que exista una instancia de gráficas activa, devolver los parámetros de
            self.plot1.clearPlotReferences()            # las gráficas a sus valores iniciales. Vease las clases 'plotWindow' y 'MplCanvas'
            self.timeAxisLimit = 1
            self.thrustAxisLimit = 3
            self.torqueAxisLimit = 1
            self.speedAxisLimit = 10000
            self.plot1.plot.axesT.set_xlim(0, 5000)
            self.plot1.plot.axesT.set_ylim(-0.1, 3)
            self.plot1.plot.axesM.set_xlim(0, 5000)
            self.plot1.plot.axesM.set_ylim(-0.1, 1)
            self.plot1.plot.axesR.set_xlim(0, 5000)
            self.plot1.plot.axesR.set_ylim(0, 10000)
            self.plot1.updatePlot([], [], 0, "T")
            self.plot1.updatePlot([], [], 0, "M")
            self.plot1.updatePlot([], [], 0, "R")
            self.plot1.plot.draw()
        
        
        
    def runSampleSweep(self):                           # Método llamado cuando el usuario presiona el botón 'RunSweep' de la barra de herramientas
                                                        # Ejecuta las siguientes acciones al ser llamado:
        if (self.comCheck() == 1):                      # -> En caso de que la conexión con el microcontrolador sea segura:
            if (self.readStatus == 0):                  # ---> En caso de que no haya una sesión de muestreo activa:
                self.recordT.append(recordedData())     # -----> Acoplar una entrada de juegos de lecturas a los vectores de almacenamiento, 
                self.recordM.append(recordedData())     #        para la nueva lectura por realizar
                self.recordR.append(recordedData())
                self.dataSets += 1                      # -----> Incrementar el contador de juegos de datos
                self.readStatus = 1                     # -----> Habilitar la Flag de sesión de muestreo activa
            self.pauseStatus = 0                        # ---> Deshabilitar la Flag de sesión en pausa
            self.initSampling()                         # ---> Llamar al método 'self.initSampling()'  El inicializador de las rutinas de muestreo en tiempo real
            
            
            
    def stopSampleSweep(self):                          # Método llamado cuando el usuario presiona el botón 'StopSweep' de la barra de herramientas
                                                        # Ejecuta las siguientes acciones al ser llamado:
        if (self.comCheck() == 1):                      # -> En caso de que la conexión con el microcontrolador sea segura:
            Arduino.write(bytes("s", 'utf-8'))          # ---> Ordenar al microcontrolador a pausar la exportación de lecturas
            self.pauseStatus = 1                        # ---> Habilitar la Flag de sesión en pausa
        self.timerSweep.stop()                          # -> Detener el temporizador de Lectura de Barrido
        
        
        
    def runSamplePeriod(self):                                                          # Método llamado cuando el usuario presiona el botón 'RunPeriod' de la barra de herramientas
                                                                                        # Ejecuta las siguientes acciones al ser llamado:
        if (self.comCheck() == 1):                                                      # -> En caso de que la conexión con el microcontrolador sea segura:
            try:                                                                        # ---> Intentar:
                self.period = int(self.samplePeriodText.text())                         # -----> Asignar los valores de 'period' y 'steps' como enteros de aquellos contenidos en 
                self.steps = int(self.sampleNumberText.text())                          #        los recuadros de texto editables por el usuario
            except:                                                                     # ---> Excepción:
                self.textEdit.append("Invalid values entered for step configuration")   # -----> Mostrar mensaje de error
                return 0                                                                # -----> Finalizar el método
                                                                                        # -----> ** La excepción se activará para cuando los valores ingresados por el usuario
                                                                                        #           no sean compatibles con el formato int
            
            if (self.period <= 0 or self.steps <= 0):                                   # -> En caso de que 'period' o 'steps' sean menores o iguales a 0
                self.textEdit.append("Invalid values entered for step configuration")   # ---> Mostrar mensaje de error
                return 0                                                                # ---> Finalizar el método
            
            self.recordT.append(recordedData())                                         # -----> Acoplar una entrada de juegos de lecturas a los vectores de almacenamiento, 
            self.recordM.append(recordedData())                                         #        para la nueva lectura por realizar
            self.recordR.append(recordedData())
            self.dataSets += 1                                                          # -----> Incrementar el contador de juegos de datos
            
            self.decreasingPeriod = self.period                                         # -> Asignar los valores de 'decreasingPeriod' y 'decreasingSteps' como aquellos de
            self.decreasingSteps = self.steps                                           #    'period' y 'steps' una vez se han determinado como válidos
            
            self.powerSteps = np.linspace(0, 65, self.steps)                            # -> Cálcular los valores de Throttle para cada etapa con base en el número de etapas
                                                                                        #    y el rango de Throttle específicado
            self.updateRPM2(self.powerSteps[0])                                         # -> Ordenar al microcontrolador a cambiar a la primer etapa de Throttle
            
            self.readStatus = 1                                                         # -> Habilitar la Flag de sesión de muestreo activa
            self.initSampling()                                                         # -> Llamar al método 'self.initSampling()'  El inicializador de las rutinas de muestreo en tiempo real
            
                
            
    def responseTestRun(self):                      # Método en desarrollo
        if (self.comCheck() == 1):
            self.mode = "Response Test"
            
    
            
    def initSampling(self):                                                         # Método llamado por 'self.runSamplePeriod()', 'self.runSampleSweep()' y 'self.responseTestRun()'
        if (self.mode == "Manual"):                                                 # Ejecuta las siguientes acciones al ser llamado:
            if (self.readStatus == 1 and self.comCheck() == 1):                     # -> En caso de que 'mode' sea 'Manual', 'readStatus' sea 1 y 'self.comCheck()' devuelva 1: 
                Arduino.write(bytes("r", 'utf-8'))                                  # ---> Ordenar al microcontrolador a solicitar y exportar lecturas y marcas temporales de los sensores
                self.timerSweep.start()                                             # ---> Activa el temporizador 'timerSweep', conectado a 'self.updateSampleSweep()'

        elif (self.mode == "Auto Period"):                                          # -> En caso de que 'mode' sea 'Auto Period', 'readStatus' sea 1 y 'self.comCheck()' devuelva 1: 
            if (self.readStatus == 1 and self.comCheck() == 1):
                Arduino.write(bytes("r", 'utf-8'))                                  # ---> Ordenar al microcontrolador a solicitar y exportar lecturas y marcas temporales de los sensores
                self.textEdit.append("")                                            # ---> Imprimir mensaje de inicio de cuenta regresiva en la consola
                text = "Countdown begin at: " + str(self.countdown) + "seconds"
                self.textEdit.insertPlainText(text)
                self.textEdit.append("")
                self.autoPeriodCountDownTimer.start()                               # ---> Iniciar el temporizador 'autoPeriodCountdownTimer', conectado a 'self.updateCountdown()'
                    
        elif (self.mode == "Response Test"):                                        # -> Sección en desarrollo
            i = 0
            # Aun no implementado
            
            
            
    def updateSampleSweep(self):                # Método llamado cuando el temporizador 'timerSweep' se encuentra en condición de timout. Ejecuta:
        s = Arduino.readline()                  # -> Leer los datos importados por el microcontrolador al buffer del puerto COM, guardar los datos en 's'
        self.updatePlotData(s)                  # -> Llamar a 'self.updatePlotData()' pasando 's' como parámetro
        
        
        
    def updateCountdown(self):                                                          # Método llamado por 'self.initSampling()' cuando 'mode' es 'Auto Period'. Ejecuta:
        if self.countdown == 0:                                                         # -> En caso de que 'countdown' sea 0:
            self.autoPeriodCountDownTimer.stop()                                        # ---> Detener temporizador 'autoPeriodCountdownTimer'
            self.countdown = self.userCountdown                                         # ---> Reinicar el contador 'countdown' al valor especificado por el usuario
            self.textEdit.append("")                                                    # ---> Mostrar mensaje de inicio de muestreo por etapas
            text = "Sampling by Step begin. Throttle at: " + str(self.powerSteps[0])    
            self.textEdit.insertPlainText(text)
            self.textEdit.append("")
            self.timerAutoPeriod.start()                                                # ---> Iniciar el temporizador 'timerAutoPeriod', conectado a 'self.updateSamplePeriod()'
        else:                                                                           # -> De lo contrario:
            self.countdown -= 1                                                         # ---> Decrementa en 1 a 'countdown'
            s = Arduino.readline()                                                      # ---> Leer los datos importados por el microcontrolador al buffer del puerto COM, guardar los datos en 's'
            self.textEdit.append("")                                                    # ---> Mostrar mensaje de cuenta regresiva
            text = "Countdown: " + str(self.countdown) + "seconds"
            self.textEdit.insertPlainText(text)
            self.textEdit.append("")
            
            
            
    def updateSamplePeriod(self):                                                               # Método llamado por 'self.updateCountdown()' cuando 'countdown' se vuelve 0
        if self.decreasingSteps >= 0:                                                           # -> En caso de que 'decresingSteps' sea mayor o igual a 0:
            self.decreasingPeriod -= self.periodSamplingInterval                                # ---> Decrementar 'decreasingPeriod' en 'periodSamplingInterval'
            s = Arduino.readline()                                                              # ---> Leer los datos importados por el microcontrolador al buffer del puerto COM, guardar los datos en 's'
            self.updatePlotData(s)                                                              # ---> Llamar a 'self.updatePlotData()' pasando a 's' como parámetro
            if (self.decreasingPeriod <= 0):                                                    # ---> En caso de que 'decreasingPeriod' sea menor o igual a 0:
                self.decreasingSteps -= 1                                                       # -----> Decrementar 'decreasingSteps' en 1
                self.decreasingPeriod = self.period                                             # -----> Reasignar el valor de 'period' (introducido por el usuario) a 'decreasingPeriod
                self.updateRPM2(self.powerSteps[self.stepIndex])                                # -----> Llamar a 'self.updateRPM2()' pasando como argumento la entrada de posición 'stepIndex' del vector 'powerSteps'
                
                self.textEdit.append("")                                                        # -----> Imprimir mensaje de cambio de etapa con su respectivo Throttle
                text = "Step Change. Throttle at: " + str(self.powerSteps[self.stepIndex])
                self.textEdit.insertPlainText(text)
                self.textEdit.append("")
                
                self.stepIndex += 1                                                             # -----> Incrementar 'stepIndex' en 1
        else:                                                                                   # ---> De lo contrario:
            Arduino.write(bytes("s", 'utf-8'))                                                  # -----> Ordenar al microcontrolador a pausar la exportación de lecturas
            self.updateRPM2(0)                                                                  # -----> Llamar a 'self.updateRPM2()' pasando como parámetro un Throttle de 0
            self.stepIndex = 0                                                                  # -----> Devolver 'stepIndex' a su valor inicial
            self.textEdit.append("Sampling by Step Done")                                       # -----> Imprimir mensaje de Muestreo por Etapas concluido
            self.timerAutoPeriod.stop()                                                         # -----> Detener el temporizador 'timerAutoPeriod'
            self.readStatus = 0                                                                 # -----> Desactivar la Flag de sesión de lectura activa
            
            
            
    def updatePlotData(self, s):                                        # Método llamado siempre que Sampler debe actualizar las lecturas en la consola con los datos recibidos del microcontrolador. Los datos que se actualizan se insertan en el parámetro 's'
        d = s.decode('utf-8')                                           # La cadena de caracteres recibida del microcontrolador esta, por defecto, codificada en el formato
        split = d.split(' ')                                            # utilizado por Python para imprimir caracteres en la consola, 'utf-8'. Python reconoce esta cadena como una
        xToAdd = np.abs(int(split[4]))                                  # serie de datos tipo 'char' que no se presta para realizar operaciones de arreglos como 'split', por lo que
        yToAdd = np.abs(float(split[2]))                                # es necesario decodificarlos del formato 'utf-8' antes de insertarlos en los buffers de las gráficas.
                                                                        # Una vez decodificados, la cadena se separa usando el carácter ' ' (espacio) como separador y se extraen
                                                                        # las partes importantes, es decir los datos, en formato 'int' y 'float' respectivamente para las lecturas de
                                                                        # los sensores y las marcas temporales.
                                                                        
        try:                                                    # Sección en desarrollo
            overlay = self.plot1.overlayData.isChecked()
        except:
            overlay = None  
        
        if "HX7T" in d:
            self.textEdit.insertPlainText(d)
            yToAdd = self.noiseProtect(yToAdd, self.recordT[self.dataSets-1], 2, -1)
            (self.recordT[self.dataSets-1], self.thrustAxisLimit, self.xMaxT, self.yMaxT) = self.updateDataBuffers(self.recordT[self.dataSets-1], xToAdd, yToAdd, overlay, self.thrustAxisLimit, "T", self.xMaxT, self.yMaxT)
            
            # En caso de que la secuencia de caracteres "HX7T" se encuentre dentro de la linea recibida del microcontrolador:
            # -> Insertar la linea recibida en la consola
            # -> Filtrar la lectura en busca de posibles afectaciones por el ruido electríco o datos incoherentes, mediante 'self.noiseProtect()'. Vease la función más abajo.
            # -> Para ofrecer una experiencia de visualización de datos en tiempo real limpia, Sampler reajusta automáticamente los límites de los ejes de las gráficas en caso de encontrar
            # -> una lectura que supere o se acerque lo suficiente a los límites actuales. Sampler además dedica un arreglo de vectores para guardar las lecturas de multiples sesiones. Este
            # -> vector se actualiza al mismo tiempo que se revisa la valides de los límites, mediante 'self.updateDataBuffers()'. A este método se le debe especificar
            # -> datos relevantes como el tipo de lectura, los datos a insertar, máximos históricos, etc. Vease el método más abajo
            
        elif "HX7M" in d:
            self.textEdit.insertPlainText(d)
            d1 = Arduino.readline()
            d1 = d1.decode('utf-8')
            s1 = d1.split(' ')
            x1 = np.abs(int(s1[4]))
            y1 = np.abs(float(s1[2]))
            MBufferX = [x1, xToAdd]
            MBufferY = [y1, yToAdd]
            self.textEdit.insertPlainText(d1)
            (promx, promy) = [np.sum(MBufferX)/2, np.sum(MBufferY)/2]
            promy = self.noiseProtect(promy, self.recordM[self.dataSets-1], 0.1, -0.1)
            
            (self.recordM[self.dataSets-1], self.torqueAxisLimit, self.xMaxM, self.yMaxM) = self.updateDataBuffers(self.recordM[self.dataSets-1], promx, promy, overlay, self.torqueAxisLimit, "M", self.xMaxM, self.yMaxM)
            
            # De lo contrario, en caso de que la secuencia de caracteres "HX7M" se encuentre dentro de la linea recibida del microcontrolador:
            # Debido a que las lecturas de torque se exportan una tras otra (hay 2 celdas dedicadas a medir el torque), Sampler guarda en una variable adicional 'd1' la 
            # lectura concecuente de la primer linea en donde "HX7M" es detectado. Despues de separar a 'd1' de la misma manera que 'd', Sampler inserta ambas lineas por
            # separado a la consola, obtiene el promedio de ambas lecturas y ambas marcas temporales, las somete al filtro anti-ruido y finalmente llama a
            # 'self.updateDataBuffers()'.
            
        elif "RPMp" in d:
            self.textEdit.insertPlainText(d)
            (self.recordR[self.dataSets-1], self.speedAxisLimit, self.xMaxR, self.yMaxR) = self.updateDataBuffers(self.recordR[self.dataSets-1], xToAdd, yToAdd, overlay, self.speedAxisLimit, "R", self.xMaxR, self.yMaxR)
            
            # En caso de que la secuencia de caracteres "RPMp" se encuentre dentro de la linea recibida del microcontrolador:
            # -> Insertar la linea recibida en la consola
            # -> Llamar a 'self.updateDataBuffers()'. La lectura de rpm's ya posee un filtro activo en el programa del microcontrolador.
            
            
    def updateDataBuffers(self, record, xToAdd, yToAdd, overlay, axisLimit, plotType, xMax, yMax):
        if record.dataCount > len(record.xData)-3:
            record.increaseSize()
        record.appendData(xToAdd, yToAdd, record.dataCount)
        (t, y) = record.verifyMaximun(xToAdd, yToAdd)
        record.dataCount += 1
        """                     # Sección en desarrollo, NO habilitar
        match overlay:
            case False:     
        """
        
        axisLimit = self.checkForRescale(plotType, y, t, axisLimit)
        self.plot1.updatePlot(record.xData[0:record.dataCount-1], record.yData[0:record.dataCount-1], 0, plotType)
        self.plot1.redraw()     
        """                         # Sección en desarrollo, NO habilitar
            case True:
                (xMax[self.dataSets], yMax[self.dataSets]) = record.verifyMaximun()
                (t, y) = [np.max(xMax), np.max(yMax)]
                axisLimit = self.checkForRescale(plotType, y, t, axisLimit)
                i = 0
                while i < self.dataSets+1:
                    self.plot1.updatePlot(record.xData, record.yData, i, plotType)
                    i = i + 1
                self.plot1.redraw()             
            case None:
                i = 0
        """
        return (record, axisLimit, xMax, yMax)                
             # self.updateDataBuffers() es llamado cuando Sampler necesita actualizar los buffers de lecturas guardados y, en caso de que existan, los buffers de las gráficas en tiempo real.
             # -> En caso de que el atributo 'dataCount' del objeto 'record' de la clase 'recordedData' pasado como parámetro a la llamada, sea mayor que el tamaño del vector
             # -> 'xData' del objeto 'record' menos 3: Incrementar el tamaño de los buffers de 'record', [vease 'recordedData::increaseSize()']. El método 'increaseSize()' de
             # -> la clase 'recordedData' básicamente concatena un vector de zeros de tamaño 100 a los vectores 'xData' y 'yData', mientras que 'dataCount' actua como indice
             # -> para que Sampler inserte las lecturas de manera secuencial sin dejar zeros en los vectores. Por ende, puede actuar tambíen como contador de cuantos datos han sido
             # -> ingresados si es incrementado en uno con cada llamada de 'self.updateDataBuffers'; cuando se vuelve mayor al tamaño de los vectores de datos menos 3, o
             # -> en otras palabras, la cantidad de datos ingresados es casi igual al tamaño del vector de datos, es necesario concatenar más entradas al vector para guardar
             # -> las lecturas futuras. Ya que Sampler incrementa el tamaño en 100 por cada llamada y la frecuencia de muestreo de las celdas es de alrededor de 11-16 muestras
             # -> por segundo, Sampler incrementara el tamaño cada 7-9 segundos, comprometiendo mínimamente el rendimiento.
             
             # -> 'record.appendData()' inserta propiamente las lecturas más recientes a los buffers del objeto 'record' con base en el indice 'dataCount', despues verifíca si los
             # -> valores de las lecturas actuales son mayores que los máximos históricos respectivos de las lecturas y devuelve los valores màs grandes en 't' y 't'. Incrementa
             # -> 'dataCount' en 1
             
             # -> 'self.checkForRescale()' revisa, de manera similar a 'record.verifyMaximun()', si los valores pasados como parámetros 't' y 'y' son mayores que los límites establecidos
             # -> para las gráficas y decide si es necesario reescalar los ejes. Vease la función debajo
             
             # -> 'plot1.updatePlot()' actualiza los buffers internos de las gráficas correspondientes a cada lectura, insertando como datos los vectores 'xData' y 'yData' de 'record'
             # -> desde 0 hasta 'dataCount' menos 1.
             
             # -> 'plot1.redraw()' actualiza los datos mostrados en el recuadro de la gráfica, indicandole al objeto que debe volver a dibujar su contenido.
             # .> Vease 'plotWindow::redraw() más abajo.
             
             # Al final, el método regresa el vector de lecturas guardadas actualizado, el límite actual del eje de lecturas correspondiente a la gráfica manejada y los valores
             # máximos actuales del record manejado
             
    

    def checkForRescale(self, plot, y, t, yAxisLimit):              # Método llamado por 'self.updateDataBuffers()'. Requiere el tipo de gráfica, valor 'y' actual, valor 't' actual y el límite de eje no temporal actual. Ejecuta;
        if self.timeAxisLimit < t:                                  # -> En caso de que el parámetro 't' sea mayor a 'timeAxisLimit', el límite del eje temporal de todas las gráficas:
            self.rescaleTime()                                      # ---> Llamar a 'self.rescaleTime()'
        if yAxisLimit < y:                                          # -> En caso de que el parámetro 'y' sea mayor a 'yAxisLimit':
            yAxisLimit = self.rescaleYAxis(y, yAxisLimit, plot)     # ---> Llamar a 'self.rescaleYAxis()'
        return yAxisLimit                                           # -> Devolver 'yAxisLimit'



    def rescaleTime(self):                                          # Método llamado por 'self.checkForRescale()':
        self.timeAxisLimit += 10000                                 # -> Incrementar 'timeAxisLimit' en 10000 (milisegundos)
        self.plot1.plot.axesT.set_xlim(0, self.timeAxisLimit)       # -> Reescalar el eje x de la gráfica de Tracción
        self.plot1.plot.axesM.set_xlim(0, self.timeAxisLimit)       # -> Reescalar el eje x de la gráfica de Torque
        self.plot1.plot.axesR.set_xlim(0, self.timeAxisLimit)       # -> Reescalar el eje x de la gráfica de velocidad angular
        self.plot1.plot.draw()                                      # -> Limpiar y dibujar contenido y ejes de todas las gráficas
    
    
    
    def rescaleYAxis(self, y, limit, plot):                         # Método llamado por 'self.checkForRescale()'. Requiere el valor 'y' que resultó mayor al límite, el límite mismo y el tipo de gráfica. Ejecuta:
        limit = y*1.1                                               # -> Asignar a 'limit' 1.1 veces el valor de 'y'
        match plot:                                                 # -> Desglose de casos: Variable 'plot' sea:
            case "T":                                               # -> Igual a "T":
                self.plot1.plot.axesT.set_ylim(-limit, limit)       # ---> Cambia los límites superior e inferior del eje "Y" de la gráfica de Tracción

            case "M":                                               # -> Igual a "M":
                self.plot1.plot.axesM.set_ylim(-limit, limit)       # ---> Cambia los límites superior e inferior del eje "Y" de la gráfica de Torque

            case "R":                                               # -> Igual a "R":
                self.plot1.plot.axesR.set_ylim(0, limit)            # ---> Cambia los límites superior e inferior del eje "Y" de la gráfica de velocidad angular

        self.plot1.plot.update()                                    # -> Actualiza todos los elementos de la gráfica
        self.plot1.plot.draw()                                      # -> Limpiar y dibujar contenido y ejes de todas las gráficas
        return limit                                                # -> Devolver 'limit'



    def noiseProtect(self, y, dataRecord, maxLim, minLim):          # Método llamado por 'self.updatePlotData()' despues de cada recepción de lecturas. 
                                                                    # Requiere el valor 'y' para analizar, el objeto dataRecord al que pertenecerá y los límites arbitrarios de tolerancia para considerar la lectura como correcta.
                                                                    # Ejecuta:
    
        if np.abs(y) > maxLim or np.abs(y) < minLim:                # -> En caso de que el valor absoluto de 'y' sea mayor a la tolerancia máxima, o menor a la tolerancia mínima:
            try:                                                    # ---> Intentar:
                y = dataRecord.yData[self.dataSets - 1]             # -----> Asignar a 'y' el último valor de lectura 'válido' guardado
            except:                                                 # ---> Excepción:
                y = 0                                               # -----> y = 0
        return y                                                    # Devolver 'y'
        
    
    
    def comCheck(self):                         # Método siempre llamado antes de que Sampler envie ordenes o reciba lecturas del microcontrolador. Ejecuta:
        ports = get_ports()                     # -> Llamar a 'get_ports()' y obtener los puertos COM disponibles conectados al computador
        global Arduino                          # -> Indicar que se trabajará con el objeto global 'Arduino'
        Arduino = findArduino(ports)            # -> Llamar a 'findArduino' y asignar el puerto COM indicado a la variable Arduino
        checkConnection(Arduino, self)          # -> Llamar a 'checkConnection()'
        if comStatus == 0:                      # -> Si 'comStatus' es 0 [comStatus es una variable global modificada cuando se llama a 'checkConnection()]:
            self.abortReadCauseConnection()     # ---> Llamar a 'self.abortReadCauseConnection()'
            return 0                            # ---> Devolver 0
        else:                                   # -> De lo contrario:
            return 1                            # ---> Devolver 1
    
    
    
    def abortReadCauseConnection(self):             # Método llamado por 'self.comCheck()' en caso de que no se haya podido establecer conexión con el microcontrolador. Ejecuta:
        self.actionReset.setEnabled(False)          # -> Inhabilita u oculta todos los botones relacionados con la interacción de Sampler con el microcontrolador
        self.actionPlot.setEnabled(False)
        self.actionRunSweep.setEnabled(False)
        self.actionStopSweep.setEnabled(False)
        self.actionRunPeriod.setEnabled(False)
        self.rpmSlider.setEnabled(False)
        self.textEdit.append("Connection error")    # -> Imprimir mensaje de error en la console
        self.readStatus = 0                         # -> Desactivar Flag de sesión de muestreo activa
        
    
        
    def modeCheck(self):                            # Método llamado por la función pública 'checkConnection()' para comunicar el modo en el que se encuentra Sampler
        if (self.mode == "Manual"):
            return "Manual Sampling"       
        elif (self.mode == "Auto Period"):
            return "Automatic Period"
        else:
            return "Not selected"
        
        
        
    def showPlot(self):                             # Método llamado por 'actionPlot' al ser presionado. Ejecuta:
        if (self.plot1 is None):                    # -> En caso de que no exista una instancia de gráficas activa:
            self.plot1 = plotWindow(self.mode)      # ---> Crear una instancia de gráficas
        self.plot1.show()                           # -> Mostrar la instancia de gráficas
        
        
        
    def updateRPM(self):                            # Método llamado por 'rpmSlider' al ser manipulado por el usuario. Ejecuta:
        if (self.comCheck() == 1):                  # -> En caso de que 'self.comCheck()' devuelva 1:
            Arduino.write(bytes("n", 'utf-8'))      # ---> Ordenar al microcontrolador a cambiar la configuración de Throttle
            rpm = self.rpmSlider.value()            # ---> Capturar el valor del slider manipulado por el usuario. La nueva configuración de Throttle solicitada por el usuario
            rpm = struct.pack("I", int(rpm))        # ---> Empaquetar la variable 'rpm' en formato Integer de 4 bytes para poder exportarla al microcontrolador
            Arduino.write(rpm)                      # ---> Enviar la nueva configuración de Throttle al microcontrolador
            if self.readStatus == 1:                # ---> En caso de que haya una sesión de muestreo activa:
                Arduino.write(bytes("r", 'utf-8'))  # -----> Ordenar al microcontrolador a resumir la exportación de lecturas
                
                
                
    def updateRPM2(self, value):                    # Método llamado por 'self.updateSamplePeriod()' y 'self.runSamplePeriod()' para cambiar automáticamente la configuración de Throttle. Requiere la configuración nueva.
                                                    # Ejecuta:
        if (self.comCheck() == 1):                  # -> En caso de que 'self.comCheck()' devuelva 1:
            Arduino.write(bytes("n", 'utf-8'))      # ---> Ordenar al microcontrolador a cambiar la configuración de Throttle
            rpm = value                             # ---> Guardar la configuración pasada como parámetro en una variable 'rpm'
            rpm = struct.pack("I", int(rpm))        # ---> Empaquetar la variable 'rpm' en formato Integer de 4 bytes para poder exportarla al microcontrolador
            Arduino.write(rpm)                      # ---> Enviar la nueva configuración de Throttle al microcontrolador
            Arduino.write(bytes("r", 'utf-8'))      # ---> Ordenar al microcontrolador a resumir la exportación de lecturas
        
        
        
        
        
        
        
def get_ports():                    # Función llamada por 'Main.comCheck()' para obtener los puertos seriales o COM disponibles en el computador
    
    ports = serialP.comports()
    
    return ports

def findArduino(ports):                         # Función llamada por 'Main.comCheck()' para encontrar el puerto COM correspondiente al microcontrolador Arduino. Ejecuta:
    
    comPort = "None"
    numConnections = len(ports)                 # -> Encontrar cuantos enlaces COM/serial existen
    global comStatus                            # -> Indicar que se trabajará con la variable global 'comStatus'
    for i in range (0, numConnections):         # -> Para 'i' desde 0 hasta 'numConnections', ejecutar:
        
        portID = str(ports[i])                  # -> Extraer el nombre del puerto número 'i'
        
        if "ACM" in portID:                     # -> Si dentro del nombre del puerto se encuentra la secuencia "ACM", [Arduino se identifica por medio de esta secuencia en Linux] ejecutar:
            comStatus = 1                       
            split = portID.split(' ')           # ---> Separar el nombre del puerto usando ' ' [espacio] como separador
            portID = split[0]                   # ---> Extraer el primer fragmento del nombre separado
            serialConfig = serial.Serial(portID, baudrate = 57600, timeout = 1)     # ---> Establecer conexión serial con Arduino, baudrate de 57600, timeout de 1 segundo
            return serialConfig                 # ---> Regresar la configuración serial recientemente iniciada
        else:                                   # -> De lo contrario:
            comStatus = 0
            
    return comPort

def checkConnection(Arduino, ui):       # Función llamada por 'Main.comCheck()' para actualizar la barra de estado de la ventana principal
    if comStatus == 0:
        ui.statusInfo.setText("Connected at: " + Arduino + "   Mode: " + ui.modeCheck())
    else:
        ui.statusInfo.setText("Connected at: " + Arduino._port + "   Mode: " + ui.modeCheck())   








class plotWindow(QWidget):              # Clase de la ventana de gráficas. Hereda de QWidget
    def __init__(self, mode):                                       # --- Inicializador de la clase --- # Requiere parámetro 'mode' relacionado con la funcionalidad de Overlay [En desarrollo]
        super().__init__()
        
        self.plot = MplCanvas(self, width=5, height=4, dpi=100)     # Parámetros para cambiar el tamaño de la ventana inicial
        
        self.overlayData = QCheckBox("Overlay plot data")           # Insertar casilla de comprobación de Overlay [Funcionalidad en desarrollo]
        self.toolbar = NavigationToolbar2QT(self.plot, self)        # Insertar barra de herramientas para navegación del gráfico
        self.plotReferenceT = [None]                                # Crear listas vacias para guardar las referencias de las lineas de datos
        self.plotReferenceM = [None]                                
        self.plotReferenceR = [None]                                
        self.updatePlot([], [], 0, "T")                             # Actualizar gráficas de Tracción, Torque y velocidad angular
        self.updatePlot([], [], 0, "M")
        self.updatePlot([], [], 0, "R")
        match mode:                                                 # Desglose de casos con base en 'mode' [En desarrollo]
            case 1:
                self.overlayData.setEnabled(False)
            case 2:
                self.overlayData.setEnabled(True)
            case _:
                self.overlayData.setEnabled(False)
        
        layout = QVBoxLayout()                                      # Layout o posicionamiento de las widgets de gráficos. En este caso, posicionamiento de arreglo cuadrado
        layout.addWidget(self.toolbar)
        layout.addWidget(self.plot)
        layout.addWidget(self.overlayData)                          # Widgets plot, toolbar y overlayData añadidas al layout
        self.setLayout(layout)                                      # Layout del contenedor de la ventana entera asignado como 'layout', incluidas las 3 widgets
        
    def updatePlot(self, x, y, i, plot):                            # Método llamado cuando se requiere añadir datos a los buffers de las gráficas. Requiere los datos para añadir, indice del juego de datos y el tipo de gráfico
                                                                    # Ejecuta:
        match plot:                                                 # -> Desglose de casos con base en 'plot':
            case "T":                                               # ---> La única diferencia entre casos es el tipo de gráfica. Las instrucciones ejecutadas son:
                if self.plotReferenceT[i] is None:                  # -----> En caso de que no exista una referencia al gráfico de Tracción actual: 
                    plot_refs = self.plot.axesT.plot(x, y, 'r')     # -------> Crear el gráfico [específicamente, el dibujo de la linea de datos] de Tracción
                    self.plotReferenceT[i] = plot_refs[0]           # -------> Guardar una referencia al gráfico creado
                else:                                               # -----> De lo contrario:
                    self.plotReferenceT[i].set_ydata(y)             # -------> Modifica los buffers de datos de la referencia al gráfico creado
                    self.plotReferenceT[i].set_xdata(x)
            case "M":
                if self.plotReferenceM[i] is None:
                    plot_refs = self.plot.axesM.plot(x, y, 'r')
                    self.plotReferenceM[i] = plot_refs[0]
                else:
                    self.plotReferenceM[i].set_ydata(y)
                    self.plotReferenceM[i].set_xdata(x)
            case "R":
                if self.plotReferenceR[i] is None:
                    plot_refs = self.plot.axesR.plot(x, y, 'r')
                    self.plotReferenceR[i] = plot_refs[0]
                else:
                    self.plotReferenceR[i].set_ydata(y)
                    self.plotReferenceR[i].set_xdata(x)
        
    def addPlotReference(self):                                     # Método llamado cuando se necesitan guardar referencias a multiples gráficos del mismo tipo: Cuando se desean gráficar multiples juegos al mismo tiempo.
        self.plotReferenceT = self.plotReferenceT + [None]          # En desarrollo
        self.plotReferenceM = self.plotReferenceM + [None]          # Concatena una entrada vacia a las listas de referencias de gráficos
        self.plotReferenceR = self.plotReferenceR + [None]
        
    def clearPlotReferences(self):                                  # Método llamado cuando se desea limpiar la información de los gráficos. Ejecuta:
        i = len(self.plotReferenceT)                                # -> Obtener el tamaño de la lista de referencias
        k = 0 
        (x, y) = [0, 0]
        while k < i:                                                # -> Mientras el indice 'k' sea menor a 'i':
            if self.plotReferenceT[k] is not None:                  # ---> Si la entrada 'k' de la lista 'plotReference' no esta vacia:
                self.plotReferenceT[k].set_xdata(x)                 # -----> Asignar un 0 a los buffers de todas las referencias a gráficos
                self.plotReferenceT[k].set_ydata(y)
                
                self.plotReferenceM[k].set_xdata(x)
                self.plotReferenceM[k].set_ydata(y)
                
                self.plotReferenceR[k].set_xdata(x)
                self.plotReferenceR[k].set_ydata(y)
            k += 1                                                  # ---> Incrementar 'k' en 1
        
    def redraw(self):                                               # Método llamado cuando los buffers de gráficos han sido actualizados y únicamente es necesario reflejar el cambio visualmente. Ejecuta:
        self.plot.axesT.draw_artist(self.plot.axesT.patch)          # -> Actualiza visualmente el espacio del gráfico
        self.plot.axesT.draw_artist(self.plotReferenceT[0])         # -> Actualiza la linea de datos graficados
        self.plot.axesM.draw_artist(self.plot.axesM.patch)
        self.plot.axesM.draw_artist(self.plotReferenceM[0])
        self.plot.axesR.draw_artist(self.plot.axesR.patch)
        self.plot.axesR.draw_artist(self.plotReferenceR[0])
        self.plot.update()                                          # Actualizar las propiedades de la figura contenedora de las gráficas
        self.plot.flush_events()                                    # Actualmente inutil para las versiones recientes de Matplotlib
        
        
        
        
        
        
        
        
        
class MplCanvas(FigureCanvasQTAgg):                                 # Clase de la figura en donde se crean, contienen y muestran las gráficas de Tracción, Torque y velocidad angular. Hereda de FigureCanvasQtAgg
    def __init__(self, parent=None, width=5, height=4, dpi=100):    # Inicializador con las dimensiones de la figura
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axesT = fig.add_subplot(131)                           # Añade un gráfico al contenedor de la figura
        self.axesT.set_ylabel("Thrust (kg)")                        # Configura las etiquetas y límites de ejes para los 3 gráficos
        self.axesT.set_xlabel("Time (ms)")
        self.axesT.set_xlim(0, 5000, True, True)
        self.axesT.set_ylim(-0.1, 3, True, True)
        
        self.axesM = fig.add_subplot(132)
        self.axesM.set_ylabel("Torque (kg*m)")
        self.axesM.set_xlabel("Time (ms)")
        self.axesM.set_xlim(0, 5000, True, True)
        self.axesM.set_ylim(-0.1, 1, True, True)
        
        self.axesR = fig.add_subplot(133)
        self.axesR.set_ylabel("Rotation Speed (rpm)")
        self.axesR.set_xlabel("Time (ms)")
        self.axesR.set_xlim(0, 5000, True, True)
        self.axesR.set_ylim(0, 10000, True, True)       
        
        super(MplCanvas, self).__init__(fig)









class recordedData:                                     # Clase de los juegos de datos que se guardan en cada sesión de muestreo.
    def __init__(self):                                 # Guarda los datos mismos, los máximos y la cuenta de cuantos datos han sido ingresados
        self.xData = []
        self.yData = []
        self.xMax = 0
        self.yMax = 0
        self.dataCount = 1
        self.increaseSize()
        
    def appendData(self, x, y, i):                      # Método para insertar nuevas lecturas en el juego de datos
        self.xData[i] = x
        self.yData[i] = y
        
    def increaseSize(self):                             # Método para concatenar un vector de 100 ceros al termino de los vectores de datos, efectivamente incrementando sus tamaños
        add = np.zeros(100);
        self.xData = np.concatenate((self.xData, add))
        self.yData = np.concatenate((self.yData, add))
        
    def verifyMaximun(self, xNew, yNew):                # Método para verificar si los datos pasados como argumentos, que idealmente son las lecturas más recientes,
        #if (abs(xNew) > abs(self.xMax)):               # son mayores que los valores máximos históricos. La sección correspondiente al máximo temporal esta deshabilitada
        #    self.xMax = xNew                           # por redundancia
        if (abs(yNew) > abs(self.yMax)):
            self.yMax = yNew
        return (xNew, self.yMax)                   # Devuelve los valores máximos

comStatus = 0                       # Flag global que indica si existe una conexión entre el computador y el microcontrolador

if __name__ == '__main__':          # Sección principal del código en donde se crean los objetos de la aplicación, asi como la interfaz, y se inicializa el bucle de sucesos
    app = QApplication(sys.argv)
    ui = Main()
    ui.show()
    app.exec_()
    
# Git State Test