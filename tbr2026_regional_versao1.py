from pybricks.hubs import PrimeHub
from pybricks.pupdevices import ColorSensor
#from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop, Axis
from pybricks.tools import wait, StopWatch
import umath, urandom
from robo import RoboTBR, MotorRobo, MotorConjunto

stopwatch = StopWatch()

class Execucao:

    def __init__(self):
        self.__tempoInicial = None
        self.__execucaoPrograma = None
        
    def validacoes(self, robotHeroes):
        angulo = robotHeroes.hub.imu.heading()
        anguloDefinido = robotHeroes.getAnguloDefinido()
        if (anguloDefinido==None):
            return [True, 0]

        if (abs(angulo-anguloDefinido)<0.25):
            return  [True, 0]
        #elif (abs(angulo-anguloDefinido)<2):
        #    return  [True, 1]
        else:
            return [False, 0]
    
    def menu(self, robotHeroes, opcaoSelecionada=None):

        while True:
            if (robotHeroes.hub.imu.ready()) and (robotHeroes.hub.imu.stationary()):          
                wait(200)
                break
            else:
                robotHeroes.hub.light.on(Color.RED)
                #robotHeroes.hub.speaker.beep(200, 100)
                wait(100)
        
        robotHeroes.hub.speaker.beep(200, 100)
        robotHeroes.hub.light.on(Color.GREEN)
        robotHeroes.hub.system.set_stop_button(Button.BLUETOOTH)
        self.menu_options = ("1","2","3","4", "5", "X")

        menu_index = 0

        while True:
            robotHeroes.hub.display.char(self.menu_options[menu_index])
            pressed = ()
            #robotHeroes.statusBateria()

            if (opcaoSelecionada==None):        
                while not pressed:
                    retValidacoes = self.validacoes(robotHeroes) 
                    if (retValidacoes[0]==False):
                        robotHeroes.hub.light.on(Color.RED)
                        #robotHeroes.hub.display.char("#")                        
                    else:
                        robotHeroes.hub.display.char(self.menu_options[menu_index])                        
                        if (retValidacoes[1]==0):
                            robotHeroes.hub.light.on(Color.GREEN)
                        elif (retValidacoes[1]==1):
                            robotHeroes.hub.light.on(Color.ORANGE)                       
                    
                    wait(5)              
                    pressed = robotHeroes.hub.buttons.pressed()
                    
                while (robotHeroes.hub.buttons.pressed()) or (opcaoSelecionada!=None):
                    wait(5)

            if (Button.CENTER in pressed) or (opcaoSelecionada!=None):
                if (opcaoSelecionada==None):
                    selecionado = self.menu_options[menu_index]
                else: 
                    selecionado = opcaoSelecionada
                    menu_index = self.menu_options.index(selecionado)
                    opcaoSelecionada = None

                if selecionado == "E" or selecionado == "X":
                    break
                else:
                    executado = False
                    robotHeroes.hub.light.on(Color.BLUE)               
                    
                    if (self.__tempoInicial==None):
                        self.__tempoInicial = stopwatch.time()

                    executado = self.problema(robotHeroes, selecionado)
                    
                    if (executado==True):
                        menu_index = (menu_index + 1) % len(self.menu_options)               
                    elif (executado!=None):
                        menu_index = self.menu_options.index(executado)              
                    else:
                        self.tempoInsuficiente(robotHeroes)
                        break
                    
                    robotHeroes.hub.light.on(Color.YELLOW)
                    
            elif Button.LEFT in pressed:
                menu_index = (menu_index - 1) % len(self.menu_options)
            elif Button.RIGHT in pressed:
                menu_index = (menu_index + 1) % len(self.menu_options)
    
    def tempoInsuficiente(self, robotHeroes):
        print("Tempo insuficiente!")
        corAlerta = [Color.GREEN, Color.YELLOW, Color.BLUE, Color.ORANGE, Color.RED] 
        robotHeroes.hub.display.char("T")
        tempo=0
        
        while (tempo<=120000):
            for i in range(len(corAlerta)):
                robotHeroes.hub.light.on(corAlerta[i])
                robotHeroes.hub.speaker.beep(200, 200)
                wait(100)
            
            tempo = stopwatch.time() - self.__tempoInicial

    def problema(self, robotHeroes, selecionado):

        tempoAtual = stopwatch.time()
        tempoInicioPrograma = tempoAtual - self.__tempoInicial
        if (tempoInicioPrograma<=120000):

            if selecionado == "1":  
                executado = executarEstrategia1(robotHeroes)
            elif selecionado == "2":
                executado = executarEstrategia2(robotHeroes, tempoInicioPrograma)
            elif selecionado == "3":
                executado = executarEstrategia3(robotHeroes, tempoInicioPrograma)
            elif selecionado == "4":
                executado = executarEstrategia4(robotHeroes, tempoInicioPrograma)
            elif selecionado == "5":
                executado = executarEstrategia5(robotHeroes, tempoInicioPrograma)
            '''
            elif selecionado == "L":
                executado = executarEstrategiaCorrecaoLixeira(robotHeroes)
            elif selecionado == "C":
                executado = executarEstrategiaCorrecaoCompleta(robotHeroes)
            '''

            tempoFinalPrograma = stopwatch.time()
            tempoExecucao = tempoFinalPrograma - tempoInicioPrograma
            tempoExecucaoTotal= tempoFinalPrograma- self.__tempoInicial
            print("Tempo de execução menu {}: {}".format(selecionado, str(tempoExecucao)))
            print("Tempo de execução total: {}".format(str(tempoExecucaoTotal)))
            return executado
        else:
            return None

    def getTempoInicial():
        return self.__tempoInicial

    def configuracoes(self, robotTBR):

        robotTBR.iniciar(anguloPartida=0, posicaoTapete=[0,0], parametroAjusteGirar=1.0, velocidadePadraoAndar=500, inLog=True)
        #robotTBR.motorLateralDireito.iniciar(gears=[16,16], direcaoPositiva=True, iniciarStalled=None)
        #robotTBR.motorLateralEsquerdo.iniciar(gears=[16,16], direcaoPositiva=False, iniciarStalled=None)
        #robotTBR.motorLateralDireito.moverParaAngulo(100)
        #robotTBR.motorLateralEsquerdo.moverParaAngulo(100)
        #robotTBR.motorCentralPrincipal.iniciar(direcaoPositiva=True, iniciarStalled=None)
        #robotTBR.motorCentralPrincipal.moverParaAngulo(1000)
        

        
        
        #robotTBR.setMotorAuxiliar(gears=[20, 12, 20, 20, 20, 20], parametroAngulo=1.70, anguloMaximo=60, direcaoPositiva=False)
        #robotTBR.andarParaTras(500)
        #return
        
        #robotTBR.subirRampa(45, 700, velocidade=500)
        
        '''
        robotTBR.andarParaFrente(200)
        robotTBR.motorLateralDireito.moverTempo(200, direcaoPositiva=False, wait=False)
        robotTBR.motorLateralEsquerdo.moverTempo(200, direcaoPositiva=False, wait=True)
        robotTBR.andarParaFrente(200)
        robotTBR.motorLateralDireito.moverTempo(100, direcaoPositiva=False, wait=False)
        robotTBR.motorLateralEsquerdo.moverTempo(100, direcaoPositiva=False, wait=True)
        robotTBR.andarParaFrente(200)
        '''
        robotTBR.motorLateralDireito.moverTempo(1000)
        robotTBR.motorLateralEsquerdo.moverTempo(2000)
        robotTBR.motorLateralConjunto.moverParaAngulo(-100)
        #robotTBR.andarParaFrente(500, velocidade=900)
        #robotTBR.girarAntiHorario(90, modo="MOTORESQUERDO")
        #robotTBR.andarParaFrente(100, velocidade=100)
        #robotTBR.motorLateralDireito.moverTempo(1000)
        #robotTBR.motorLateralEsquerdo.moverTempo(2000)
        #robotTBR.motorLateralDireito.moverParaAngulo(100)
        #robotTBR.motorLateralEsquerdo.moverParaAngulo(100)
        

        return True




def executarEstrategia1(robotHeroes):
    '''
    Empurrar a caixa

    '''

    robotHeroes.iniciar(anguloPartida=0, posicaoTapete=[0,0], parametroAjusteGirar=1.0, velocidadePadraoAndar=500, inLog=True)
    
    '''
    robotHeroes.motorLateralEsquerdo.moverParaAngulo(-200)
    robotHeroes.motorLateralEsquerdo.moverParaAngulo(-200)

    robotHeroes.motorLateralEsquerdo.zerarAngulo()
    robotHeroes.motorLateralDireito.zerarAngulo()
    '''

    
    robotHeroes.andarParaFrente(210, velocidade=500)
    robotHeroes.girarAntiHorario(30)
    robotHeroes.andarParaFrente(210, velocidade=500)
    #robotHeroes.motorLateralEsquerdo.moverParaAngulo(135)
    robotHeroes.girarHorario(55, velocidade=150)
    robotHeroes.andarParaFrente(480)
    robotTBR.girarAntiHorario(45, velocidade=300)
    robotHeroes.andarParaFrente(150, velocidade=300)
    
    #robotTBR.motorLateralEsquerdo.moverParaAngulo(-130)
    #robotTBR.motorLateralDireito.moverParaAngulo(120)
    
    return True
    robotTBR.andarParaFrente(50)
    robotTBR.girarAntiHorario(40, velocidade=300)
    robotTBR.andarParaTras(80, velocidade=500)
    robotTBR.motorLateralDireito.moverParaAngulo(-120)
    robotTBR.girarHorario(60)
    robotTBR.andarParaFrente(400)
    robotTBR.girarAntiHorario(45)
    robotTBR.andarParaFrente(480, velocidade=150)
    robotTBR.andarParaTras(410)
    robotTBR.girarHorario(45+15)
    robotTBR.andarParaFrente(350)
    robotTBR.motorLateralEsquerdo.moverParaAngulo(20)
    robotTBR.girarHorario(90)
    

    #robotTBR.motorLateralDireito.iniciar(gears=[16,16], direcaoPositiva=True, iniciarStalled=None)
    #robotTBR.motorLateralEsquerdo.iniciar(gears=[16,16], direcaoPositiva=False, iniciarStalled=None)
    #robotTBR.motorLateralDireito.moverParaAngulo(100)
    #robotTBR.motorLateralEsquerdo.moverParaAngulo(100)
    #robotTBR.motorCentralPrincipal.iniciar(direcaoPositiva=True, iniciarStalled=None)
    #robotTBR.motorCentralPrincipal.moverParaAngulo(1000)
    

    
    
    #robotTBR.setMotorAuxiliar(gears=[20, 12, 20, 20, 20, 20], parametroAngulo=1.70, anguloMaximo=60, direcaoPositiva=False)
    #robotTBR.andarParaTras(500)
    #return
    
    #robotTBR.subirRampa(45, 700, velocidade=500)
    
    '''
    robotTBR.andarParaFrente(200)
    robotTBR.motorLateralDireito.moverTempo(200, direcaoPositiva=False, wait=False)
    robotTBR.motorLateralEsquerdo.moverTempo(200, direcaoPositiva=False, wait=True)
    robotTBR.andarParaFrente(200)
    robotTBR.motorLateralDireito.moverTempo(100, direcaoPositiva=False, wait=False)
    robotTBR.motorLateralEsquerdo.moverTempo(100, direcaoPositiva=False, wait=True)
    robotTBR.andarParaFrente(200)
    '''
    '''
    robotTBR.motorLateralDireito.moverTempo(1000)
    robotTBR.motorLateralEsquerdo.moverTempo(2000)
    robotTBR.motorLateralConjunto.moverParaAngulo(-100)
    '''
    #robotTBR.andarParaFrente(500, velocidade=900)
    #robotTBR.girarAntiHorario(90, modo="MOTORESQUERDO")
    #robotTBR.andarParaFrente(100, velocidade=100)
    #robotTBR.motorLateralDireito.moverTempo(1000)
    #robotTBR.motorLateralEsquerdo.moverTempo(2000)
    #robotTBR.motorLateralDireito.moverParaAngulo(100)
    #robotTBR.motorLateralEsquerdo.moverParaAngulo(100)
    

    return True

def executarEstrategia2(robotTBR, tempo):
    #Deixar

    robotTBR.iniciar(anguloPartida=0, posicaoTapete=[0,0], parametroAjusteGirar=1.18, velocidadePadraoAndar=500, inLog=True)
    
    #robotTBR.motorLateralEsquerdo.moverParaAngulo(160, wait=False)
    #robotTBR.motorLateralDireito.moverParaAngulo(160, wait=True)
    #robotTBR.motorLateralConjunto.moverParaAngulo(160)
    
    robotTBR.andarParaFrente(352, velocidade=400)
    robotTBR.girarAntiHorario(43)
    robotTBR.andarParaFrente(303, velocidade=400)
    robotTBR.andarParaFrente(200, velocidade=100)
    robotTBR.girarHorario(43)
    robotTBR.andarParaFrente(100, velocidade=400)
    
    return True
    
    #robotTBR.motorLateralDireito.moverParaAngulo(160, wait=False)
    #robotTBR.motorLateralDireito.moverParaAngulo(160, wait=False)
    return True
    
    robotTBR.andarParaFrente(680, velocidade=500)
    robotTBR.girarHorario(90)
    robotTBR.andarParaFrente(145, velocidade=500)
    robotTBR.motorLateralDireito.moverParaAngulo(160)
    robotTBR.andarParaTras(200, velocidade=200)
        
def executarEstrategia3(robotTBR):

    robotTBR.iniciar(anguloPartida=0, posicaoTapete=[0,0], parametroAjusteGirar=1.0, velocidadePadraoAndar=500, inLog=True)
    
    
    robotTBR.andarParaFrente(680, velocidade=500)
                  
def executarEstrategia4(robotTBR):

    robotTBR.iniciar(anguloPartida=0, posicaoTapete=[0,0], parametroAjusteGirar=1.0, velocidadePadraoAndar=500, inLog=True)
    
    
    robotTBR.andarParaFrente(680, velocidade=500)

def executarEstrategia5(robotTBR):

    robotTBR.iniciar(anguloPartida=0, posicaoTapete=[0,0], parametroAjusteGirar=1.0, velocidadePadraoAndar=500, inLog=True)
    
    
    robotTBR.andarParaFrente(680, velocidade=500)
    

robotTBR=RoboTBR()
exec = Execucao()
#exec.configuracoes(robotTBR)

exec.menu(robotTBR, "2")

