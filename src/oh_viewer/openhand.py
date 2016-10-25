#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

from __future__ import with_statement # for python 2.5
__copyright__ = 'OpenHand 2016'

import sys, os, re, logging, signal
from multiprocessing import Process,Pipe
from threading import Thread
from openravepy import *
#from mlabwrap import mlab
from numpy import *
from math import *
import time
import scipy
import copy
import random

import h5py

import os
import shutil

from Ui_OpenHand import *
#from Ui_OpenHand_Graph import *
from PyQt4 import QtGui, QtCore, Qt
import ImageQt, popplerqt4
import oh_utils.oh_utils as oh_utils

logger = None

#Define Paths
path_hand=os.path.dirname(os.path.abspath(__file__))+'/../../hands/'     
path_obj=os.path.dirname(os.path.abspath(__file__))+"/../../objects/"
path_experiments=os.path.dirname(os.path.abspath(__file__))+"/../../database/"
experiment="Default"



#Define Global Types
dt_array = h5py.special_dtype(vlen=numpy.dtype('float'))

class CallbackHandler(QtCore.QThread):
    def __init__(self,pipe,callback=None):
        super(CallbackHandler,self).__init__()
        self.callback = callback
        self.pipe = pipe

    def run(self):
        while(True):
            #print 'CallbackHandler:run'
            resultValue = self.pipe.recv()
            
            msg = [self.callback,resultValue]
            aux = self.emit(QtCore.SIGNAL("CallbackHandler(PyQt_PyObject)"),msg)
        
class OpenRaveServer(object):
    '''
    classdocs
    '''

    def __init__(self,pipe):
        '''
        Constructor
        '''
        self.pipe = pipe
        self.running = True
        self.orenv = Environment()
        self._run()
        


    def __del__(self):
        RaveDestroy()

    def _run(self):
        while(self.running):
            (functionName,args,handleCallback) = self.pipe.recv()
            rValue,rMessage = self.executeFunction(functionName, args)
            if handleCallback:
                self.pipe.send([rValue,rMessage])

    def executeFunction(self,name,args):
        rValue = None
        rMessage = "Function with "+name+" not available"
        if name in dir(self):
            if(args is None):
                rValue,rMessage = getattr(self,name)()
            else:
                rValue,rMessage = getattr(self,name)(args)
        return rValue,rMessage

    def StartGui(self):
        try:
            self.orenv.SetViewer('qtcoin')
            RaveSetDebugLevel(DebugLevel.Error)
            server = RaveCreateModule(self.orenv,'textserver')
            port = 4765
            while(self.orenv.AddModule(server,str(port))==-1):
                port+=1
            print "using port: "+str(port)            
            return True,None
        except Exception as e:
            print e
            pass
        return None,"Please start OpenRAVE first."

    def LoadEnv(self,env):
        try:
            self.orenv.Reset()
            if((env is not None) and self.orenv.Load(env) == False):
                return None, "Could not load "+env+"."
            return True,None
        except Exception as e:
            print e
            pass
        return None, "Please start OpenRAVE first."
    
    def LoadHand(self, hand):
        try:
            self.orenv.Reset()
            if (hand is not None):                
                self.robot=self.orenv.ReadRobotURI(hand)
                self.orenv.Add(self.robot, True) 
                self.hand = self.orenv.GetRobots()[0]
            return True,None
        except Exception as e:
            print e
            pass
        return None, "Please start OpenRAVE first."    
    
    def LoadObject(self,meshName):
        return oh_utils.LoadObject(self.orenv, meshName, path_obj), None
            
    def Grasp(self, param=None):  
        result=oh_utils.Grasp(self.orenv)
        if result==None:
            return False, None
        return True, result
        
    def LoadPosture(self, params):
        posture=[]
        posture.append(params[0])
        posture.append(params[1:])
        return oh_utils.LoadPosture(self.orenv, posture), None
    
    def quit(self):
        RaveDestroy()
        self.running = False
        return True,None

class Server(object):
    '''
    Control server to run the benchmark in its own process.
    '''

    def __init__(self):
        '''
        Setup the shared memory data structure model and initialize the control parts.
        '''        
        self.openRaveServers = []
        self.running = True
        self.orgui = None
        self.qtgui = None
        self.object = None
        (self.pipeQtControl, self.pipeORControl) = Pipe()
        (self.pipeQtServer, self.pipeServer) = Pipe()
        self.StartQtGuiControl()
        self._run()
        
    def _run(self):
        '''
        Main server loop which waits for input from the qt-gui.
        '''
        while(self.running):
            (functionName,args,handleCallback) = self.pipeServer.recv()      
            rValue = self.executeFunction(functionName, args)
            if handleCallback:
                self.pipeServer.send([rValue, rValue])

    def executeFunction(self,name,args):
        if name == "___START_OPENRAVE_SERVER___":
            return self.StartOpenRaveGuiServer()
        if name == "___CLOSE___":
            self.running = False
            try:
                self.qtgui.terminate()
                self.orgui.terminate()
            except:
                pass
            return True
        return False   



    def norm(self, value, max, min):
        return (value-min)/(max-min)
    
    def normalize(self, measures):
        minU=[0.0, 0.0, 0.0, 0.07, 0.0, 0.27, 0.0, 0.0, 0.51, 0.0, 0.81, 0.08]
        maxU=[0.52, 11.30, 0.24, 1.0, 0.36, 1.0, 0.09, 0.03, 0.94, 0.02, 1.0, 1.0]
        res=[]        
        for i in xrange(len(measures)):  
            measures[i]=self.norm(measures[i][0], maxU[i%10], minU[i%10])
        return measures  
    
    def normalizeRobot(self, measures):
        minU=[0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.0, 0.43, 0.0]
        maxU=[0.98, 11.30, 0.57, 1.0, 1.63, 1.0, 0.1, 0.06, 1, 0.08]
        res=[]        
        for i in xrange(len(measures)):  
            measures[i]=self.norm(measures[i][0], maxU[i%10], minU[i%10])
        return measures        
        
    def StartOpenRaveGuiServer(self):
        if self.orgui:
            self.orgui.terminate()
        self.orgui = Process(target=OpenRaveServer,args=(self.pipeORControl,))
        self.orgui.start()
        return True

    def StartQtGuiControl(self):
        self.qtgui = Process(target=self._StartQtGuiControl)
        self.qtgui.start()
        return True

    def _StartQtGuiControl(self):
        app = QtGui.QApplication(sys.argv)
        form = MainWindow(self.pipeQtControl,self.pipeQtServer)
        form.show()
        app.exec_()

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    _fromUtf8 = lambda s: s


class MainWindow(QtGui.QMainWindow, Ui_OpenHand):

    @QtCore.pyqtSignature("")
    def on_pbClose_clicked(self):
        self.closeAll()

    def __init__(self,pipeOR,pipeServer):
        super(MainWindow,self).__init__(None)
        self.pipeServer = pipeServer
        self.pipeOR = pipeOR

        
        
        self.CallbackHandler = CallbackHandler(self.pipeOR)
        self.connect(self.CallbackHandler,QtCore.SIGNAL("CallbackHandler(PyQt_PyObject)"),self.HandleCallback)
        self.CallbackHandlerServer = CallbackHandler(self.pipeServer)
        self.connect(self.CallbackHandlerServer,QtCore.SIGNAL("CallbackHandler(PyQt_PyObject)"),self.HandleCallback)
        self.__index = 0
        self.setupUi(self)        
        
        #Disable Buttons Robot
        self.pbRLoadHand.setEnabled(False)
        self.pbRLoadObject.setEnabled(False)        
        self.pbRInitialPosture.setEnabled(False)
        self.pbRGrasp.setEnabled(False)
        
        #Load ComboBoxes
        self.directory = QtCore.QDir("~/")        
        self.directory.setPath(path_experiments);      
        self.cBRLoadExp.clear()
        for item in self.directory.entryList()[2:]:
            if os.path.isfile(path_experiments+item):
                self.cBRLoadExp.addItem(item)
        
        #start OR automatically the first time
        self.updateTeOutput("Starting OpenRave.")
        self.SendToServer("___START_OPENRAVE_SERVER___")
        self.updateTeOutput("Starting qtcoin gui.")
        self.SendToOR("StartGui")      
        
                
        #Default values        
        self.actual=0
        self.postures=[]
        self.transform=[]
        self.pos=""
        self.hdf5file=None
        
    def close(self):
        self.SendToOR("___CLOSE___")
        self.SendToServer("___CLOSE___")
        QtGui.QApplication.quit()

    def closeEvent(self, event):
        self.close()
        

    @QtCore.pyqtSignature("")
    def on_pbStartOpenRAVE_clicked(self):
        self.updateTeOutput("Starting OpenRave.")
        self.SendToServer("___START_OPENRAVE_SERVER___")
        self.updateTeOutput("Starting qtcoin gui.")
        self.SendToOR("StartGui")
        self.clearWindow()
    
    @QtCore.pyqtSignature("")
    def on_pbRInitialPosture_clicked(self):
        posture_num=int(str(self.cBRInitialPosture.currentText()).split()[1])
        posture=self.obj_grp["grasps"][posture_num]
        self.actual=posture_num
        self.SendToOR("LoadPosture", posture, None)        
        self.updateTeOutput("Loaded Posture %d"%(posture_num))
        self.pbRGrasp.setEnabled(True)

    @QtCore.pyqtSignature("")
    def on_pbRGrasp_clicked(self): 
        self.ini=time.time()
        self.cleanValues()
        self.SendToOR("Grasp", None, self.updateMeasures)        
                
    @QtCore.pyqtSignature("")
    def on_pbRLoadObject_clicked(self): 
        meshName = str(self.cBRLoadObject.currentText())
            
        self.SendToOR("LoadObject",meshName)    
        
        self.obj_grp=self.hdf5file[meshName]
        self.meshName=meshName         
        self.cBRInitialPosture.clear()
        grasps_count=len(self.obj_grp['grasps'])
        posturesList=[]
        for i in range(grasps_count):
            posturesList.append("Posture %d"%i)            
        self.cBRInitialPosture.addItems(posturesList)
        self.updateTeOutput("Object %s has %d grasps"%(meshName, grasps_count))        
        self.pbRInitialPosture.setEnabled(True)        
                    
    @QtCore.pyqtSignature("")
    def on_pbRLoadExp_clicked(self): 
        expName = str(self.cBRLoadExp.currentText())        
        
        self.experiment=expName
        if self.hdf5file!=None:self.hdf5file.close()
        
        self.hdf5file=h5py.File(path_experiments+expName, 'r')
        self.updateTeOutput("Loaded experiment "+expName)
        self.cBRSelHand.clear()
        self.cBRSelHand.addItems(self.hdf5file.keys())
        self.updateTeOutput("Experiment %s has %d hands"%(expName, len(self.hdf5file.keys())))
        self.pbRLoadHand.setEnabled(True)        
        
    @QtCore.pyqtSignature("")
    def on_pbRLoadHand_clicked(self): 
        envName = str(self.cBRSelHand.currentText())  
        handName=envName      
        if "shadow" in handName:
            path = path_hand+handName+".dae"
        else:
            path = path_hand+handName+".robot.xml"        
        
        self.SendToOR("LoadHand",path)
        self.updateTeOutput("Loaded hand "+handName)
        self.hdf5file.close()
        
        self.hdf5file=h5py.File(path_experiments+self.experiment[:-5]+'/'+envName+".hdf5", "r")
        self.handName=handName
        self.cBRLoadObject.clear()
        self.cBRLoadObject.addItems(self.hdf5file.keys())
        self.updateTeOutput("Hand %s has been evalauted with %d objects"%(handName, len(self.hdf5file.keys())))
        self.pbRLoadObject.setEnabled(True)                   
                
    def updateMeasures(self, args=None):   
        self.updateTeOutput("Grasp Done")
        metrics=self.obj_grp["quality"][self.actual]       
        
        self.textA1G.setText(str("%.4f"%(metrics[0])))
        self.textA2G.setText(str("%.4f"%(metrics[1])))
        self.textA3G.setText(str("%.4f"%(metrics[2])))
        self.textB1G.setText(str("%.4f"%(metrics[3])))        
        self.textB2G.setText(str("%.4f"%(metrics[4])))
        self.textB3G.setText(str("%.4f"%(metrics[5])))
        self.textC1G.setText(str("%.4f"%(metrics[6])))
        self.textC2G.setText(str("%.4f"%(metrics[7])))
        self.textD1G.setText(str("%.4f"%(metrics[8])))
        self.textD2G.setText(str("%.4f"%(metrics[9])))        
        
           
    def cleanValues(self):
        self.textA1G.setText("")
        self.textA2G.setText("")
        self.textA3G.setText("")
        self.textB1G.setText("")
        self.textB2G.setText("")
        self.textB3G.setText("")
        self.textC1G.setText("")
        self.textC2G.setText("")
        self.textD1G.setText("")
        self.textD2G.setText("")        
                  
    def updateTeOutput(self,text):
        content = str(text)
        content += "\n"
        content += str(self.teOutput.toPlainText())
        self.teOutput.setText(content)

    def CallbackOR(self,args):
        self.ButtonsUnlock()

    def SendToServer(self,command,args=None,callback=None):
        handleCallback=False
        if callback:
            self.CallbackHandlerServer.callback = callback
            self.CallbackHandlerServer.start()
            handleCallback=True
        self.pipeServer.send([command,args,handleCallback])

    def SendToOR(self,command,args=None,callback=None):
        handleCallback=False
        if callback:
            self.CallbackHandler.callback = callback
            self.CallbackHandler.start()
            handleCallback=True
        self.pipeOR.send([command,args,handleCallback])

    def HandleCallback(self,msg):
        if(len(msg) == 2):
            if(msg[0] is not None):
                try:
                    msg[0](msg[1][1])
                except Exception as e:
                    logger.error(str(e))
            else:
                self.updateTeOutput("ERROR: "+msg[1][0])
            return
        logger.error("ERROR in request format")
        
        
def main(env,options):
    "Main example code."
    
    global logger
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    logger = logging.getLogger('PyqtControl')
    lhandler =logging.StreamHandler(sys.stdout)
    lhandler.setFormatter(logging.Formatter("%(levelname)-10s:: %(filename)-20s - %(lineno)4d :: %(message)s"))
    logger.setLevel(logging.INFO)
    logger.addHandler(lhandler)
    server = Server()
    

from optparse import OptionParser

def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Pyqt example to demonstrate how openrave elements can be controlled by a qt-gui.', usage='openrave.py --example qtserverprocess [options]')
    (options, leftargs) = parser.parse_args(args=args)
    main(None,options)

if __name__ == "__main__":
    run()
