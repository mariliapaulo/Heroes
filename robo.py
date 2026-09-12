from pybricks.hubs import PrimeHub
#from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.pupdevices import Motor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop, Axis
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
import umath, urandom


class MotorRobo:

    def __init__(self, porta, direcao=Direction.CLOCKWISE, potencia=100, velocidadePadrao=400, inLog=False):
        self.__porta = porta
        self.__direcao = direcao
        self.__potencia = potencia
        self.__velocidadePadrao = velocidadePadrao
        self.__motor = Motor(self.__porta, positive_direction=self.__direcao, gears=[20,20])
        self.__inLog = inLog
        self.__parametroAngulo = 1    
        self.__anguloMaximo = 360
        self.__angulo = 0
        
        if (self.__direcao==Direction.COUNTERCLOCKWISE):
            self.__potencia = self.__potencia * -1

    def getPorta(self):
        return self.__porta
    
    def setInLog(self, inLog):
        self.__inLog = inLog

    def iniciar(self, gears=None, parametroAngulo = 1, velocidadePadrao = None, anguloInicial = 0, anguloMaximo=180, direcaoPositiva=True, iniciarStalled=None, toleranciaErro =[10,10]):
        self.__parametroAngulo = parametroAngulo    
        
        if (velocidadePadrao!=None):
            self.__velocidadePadrao = velocidadePadrao
        
        self.__angulo = anguloInicial
        self.__anguloMaximo = anguloMaximo
        
        self.__direcao = Direction.CLOCKWISE
        if (direcaoPositiva==False):
            self.__direcao = Direction.COUNTERCLOCKWISE
            self.__potencia = self.__potencia * -1
        
        try:
            self.__motor.close()
            self.__motor = Motor(self.__porta, positive_direction=self.__direcao, gears=gears)
            self.__motor.dc(self.__potencia) 
            #self.__motor.control.target_tolerances(toleranciaErro[0],toleranciaErro[1])          #tolerância de desvio para velocidade e posição
            if (iniciarStalled==None):
                self.__motor.stop()
                self.__motor.reset_angle(self.__angulo)
            else:
                sentido = 1
                if (iniciarStalled==False):
                    sentido = -1

                self.__motor.run_angle(100, sentido*200, wait=False)
                while(self.__motor.stalled()==False):
                    continue
                if (self.__motor.stalled()):
                    self.__motor.stop()
                    self.__motor.reset_angle(self.__angulo)
                    
        except Exception as error:
            print(error)
            mensagemErro = "Erro na inicialização do motor"
            print(mensagemErro)
    
    def moverParaAngulo(self, anguloDesejado, wait=True, rangeErro=0, riscoStalled=False, tratamentoStalled=[None,None,None,None]):
        if (anguloDesejado==self.__angulo):
            return

        direcaoMovimentar = True
        if (anguloDesejado<0):
            direcaoMovimentar = False
            anguloDesejado=anguloDesejado*-1
        
        tempoMovimentar = (1400/360)*anguloDesejado
        if (anguloDesejado<45):
            tempoMovimentar = tempoMovimentar + 400
        elif (anguloDesejado<90):
            tempoMovimentar = tempoMovimentar + 400

        print (tempoMovimentar)
        self.moverTempo(tempoMovimentar, velocidade=360, direcaoPositiva=direcaoMovimentar)
        self.__angulo = anguloDesejado

    
    def _moverParaAngulo(self, anguloDesejado, velocidade=None, wait=True, rangeErro=0, riscoStalled=False, tratamentoStalled=[None,None,None,None]):
        '''
        tratamentoStalled=['Andar para Frente','Andar para Tras','Girar Horário','Girar Anti Horário']
        '''
        
        if (anguloDesejado==self.__angulo):
            return

        if (self.__angulo!=None):
            #ângulo desejado negativo, automaticamente riscoStalled
            if (anguloDesejado<0):
                riscoStalled = True
                tratamentoStalled=[None,None,None,None]
            elif (anguloDesejado>self.__anguloMaximo):
                anguloDesejado = self.__anguloMaximo
        
        if (velocidade==None):
                velocidade = self.__velocidadePadrao

        anguloMovimentar = (anguloDesejado - (self.__motor.angle()/self.__parametroAngulo))*self.__parametroAngulo 
        
        if (self.__inLog):
            print("Angulo atual motor {}, angulo real do motor {}".format(str(self.__angulo), str(self.__motor.angle())))
            print("Log Garra: Ângulo atual={}/ Ângulo desejado={}/ Ângulo movimentar={}".format(str(self.__angulo), str(anguloDesejado), str(anguloMovimentar)))
        
        terminoMovimento = self.__moverMotorComandoAngle(anguloMovimentar, velocidade, wait, riscoStalled)

        if (terminoMovimento==False) and (riscoStalled):
            self.__tratamentoStalled(tratamentoStalled)

        self.__angulo = anguloDesejado
        if (self.__angulo<0):
            self.__angulo=0
            self.__motor.reset_angle(0)
            
    def __moverMotorComandoAngle(self, anguloMovimentar, velocidade, wait=True, riscoStalled=False):        
        self.__motor.run_angle(velocidade, anguloMovimentar, wait=wait)
        if (riscoStalled):
            wait=True
        
        if (wait):
            while(self.__motor.stalled()==False):
                if (self.__motor.done()): 
                    self.__motor.stop()    
                    return True
            if (riscoStalled):
                return False
            else:
                return True
        else:
            return True

    def __tratamentoStalled(self, tratamentoStalled):        
        return True
        
    def moverTempo(self, tempo=None, velocidade=None, direcaoPositiva=True, wait=True, riscoStalled=False, tratamentoStalled=[None,None,None,None]):
        if (velocidade==None):
            velocidade = self.__velocidadePadrao

        if (direcaoPositiva==False):
            velocidade = velocidade*-1

        if (tempo==None): #sem tempo, mover até um comando parar
            self.__motor.run(velocidade)
            return True
        elif (wait==False):
            self.__motor.run_time(velocidade, tempo, wait=False)
            return True            
        elif (wait):
            if (riscoStalled):
                self.__motor.run_time(velocidade, tempo, wait=False)
                while(self.__motor.stalled()==False):
                    if (self.__motor.done()): 
                        self.__motor.stop()    
                        return True
                return False
            else:
                self.__motor.run_time(velocidade, tempo, wait=True)
                self.__motor.stop()   
                return True 

    def parar(self):
        self.__motor.stop()
    
    def done(self):
        self.__motor.done()

    def zerarAngulo(self, velocidade=None):
        if (velocidade==None):
            velocidade=self.__velocidadePadrao

        anguloMovimentar=self.__angulo*1.2*self.__parametroAngulo*-1
        
        self.__motor.run_angle(velocidade, anguloMovimentar, wait=False)
        while(self.__motor.stalled()==False):
            if (self.__motor.done()): 
                break    
        if (self.__inLog):
            if (self.__motor.stalled()):
                print("Stalled em zerar motor.")
        self.__motor.stop()            
        self.__motor.reset_angle(0)
        self.__angulo=0

    
        
        self.__angulo=0

class MotorConjunto:
    def __init__(self, motor1, motor2, inLog=False):
        self.__motor1 = motor1
        self.__motor2 = motor2
        self.__inLog = inLog
        
    def moverParaAngulo(self, anguloDesejado, velocidade=None, inZerarAngulo=True):
        if (inZerarAngulo):
            self.__motor1.zerarAngulo()
            self.__motor2.zerarAngulo()
        
        self.__motor1.moverParaAngulo(anguloDesejado, wait=False)
        self.__motor2.moverParaAngulo(anguloDesejado, wait=True)

        self.__motor1.parar()
        self.__motor2.parar()
        
    def setInLog(self, inLog):
        self.__inLog = inLog
    

class RoboTBR:

    def __init__(self, diametroRoda=None, distanciaEntreRodas=None, distanciaCentroPonta=None, topSideHub=Axis.Z, frontSideHub=-Axis.X, inSensor=False, inTratamentoErro=False, inLog=False):        
        
        if (diametroRoda==None):
            #Configuraçoes padrões, roda pequena 56.0, roda grande 88.0
            diametroRoda = 86.3
            
        if (distanciaEntreRodas==None):
            distanciaEntreRodas = 160
            
        
        if (distanciaCentroPonta==None):
            distanciaCentroPonta = 60

        self.__inTratamentoErro = inTratamentoErro
        self.__inLog = inLog
        self.__anguloDefinido = None

        #Configuração do Hub
        #Instruções em
        self.hub = PrimeHub(top_side=topSideHub, front_side=frontSideHub)
        self.hub.display.orientation(self.hub.imu.up())
        
        self.__portaMotorRodaEsquerda = Port.F
        self.__portaMotorRodaDireita = Port.B
        
        #Definições dos motores das rodas
        try:
            #Hub está de cabeça pra baixo, então o motor que visualmente é esquerdo é o direito para o hub e vice-versa        
            self.__motorRodaEsquerda = Motor(self.__portaMotorRodaEsquerda, positive_direction=Direction.COUNTERCLOCKWISE)
            self.__motorRodaDireita = Motor(self.__portaMotorRodaDireita)
        except Exception as error:
            print(error)
            mensagemErro = "Erro na inicialização dos motores"
            print(mensagemErro)
            return

        #Definição do Robo
        self.robot = DriveBase(self.__motorRodaEsquerda, self.__motorRodaDireita, wheel_diameter=diametroRoda, axle_track=distanciaEntreRodas)
        
        #Definição dos outros motores
        self.motorCentralPrincipal  = None
        self.motorCentralAuxiliar  = None
        self.motorCentralConjunto = None
        self.motorLateralDireito  = None
        self.motorLateralEsquerdo  = None
        self.motorLateralConjunto = None
        
        self.__portaMotorLateralDireito = Port.A
        self.__portaMotorLateralEsquerdo = Port.E
        self.__portaMotorCentralPrincipal = Port.C
        
        try:
            self.motorLateralDireito  = MotorRobo(self.__portaMotorLateralDireito, direcao=Direction.COUNTERCLOCKWISE, inLog=self.__inLog)
            self.motorLateralEsquerdo  = MotorRobo(self.__portaMotorLateralEsquerdo, direcao=Direction.CLOCKWISE, inLog=self.__inLog)
            self.motorLateralConjunto = MotorConjunto(self.motorLateralEsquerdo, self.motorLateralDireito, inLog=self.__inLog)
            self.motorCentralPrincipal = MotorRobo(self.__portaMotorCentralPrincipal, inLog=self.__inLog)
            
        except Exception as error:
            print(error)
            mensagemErro = "Erro na inicialização dos motores laterais"
            print(mensagemErro)
        
        #Configurações dos comandos straight, turn e curve(?)
        #Padrão (195, 733, 186, 840) - Velocidade padrão do spike        
        
         
        self.__velocidadePadraoAndar = 400      #mm/s2
        self.__aceleracaoPadraoAndar = 733      #mm/s2
        self.__velocidadePadraoGirar = 500      #mm/s
        self.__aceleracaoPadraoGirar = 840      #mm/s2
        self.robot.settings(self.__velocidadePadraoAndar, self.__aceleracaoPadraoAndar, self.__velocidadePadraoGirar, self.__aceleracaoPadraoGirar)
        #self.robot.heading_control.target_tolerances(5, 5)  #tolerância de desvio para velocidade e posição
        #self.robot.distance_control.target_tolerances(5, 5)            

        self.__velocidadeAndarAtual=self.__velocidadePadraoAndar
        self.__aceleracaoAndarAtual=self.__aceleracaoPadraoAndar
        self.__velocidadeGirarAtual=self.__velocidadePadraoGirar
        self.__aceleracaoGirarAtual=self.__aceleracaoPadraoGirar

        #Definição dos sensores
        self.__ultraSonicSensor = None
        self.__forceSensor = None
        self.__colorSensor1 = None
        self.__colorSensor2 = None

        if (inSensor):
            vetPortasLivres = []
            vetPortasLivres.append(Port.B)
            vetPortasLivres.append(Port.F)

            for i in range(len(vetPortasLivres)):
                try:
                    porta = vetPortasLivres[i]
                    self.__ultraSonicSensor=UltrasonicSensor(porta)
                    print("Ultrasonic sensor -> " + str(porta))
                    vetPortasLivres.remove(porta)
                    break
                except:
                    continue
            
            for i in range(len(vetPortasLivres)):
                try:
                    porta = vetPortasLivres[i]
                    self.__forceSensor=ForceSensor(porta)
                    print("Force sensor -> " + str(porta))
                    vetPortasLivres.remove(porta)
                    break
                except:
                    continue
            
            for i in range(len(vetPortasLivres)):
                try:
                    porta = vetPortasLivres[i]
                    self.__colorSensor1=ColorSensor(porta)
                    print("Color sensor -> " + str(porta))
                    vetPortasLivres.remove(porta)
                    break
                except:
                    continue
            
            for i in range(len(vetPortasLivres)):
                try:
                    porta = vetPortasLivres[i]
                    self.__colorSensor2=ColorSensor(porta)
                    print("Color sensor 2 -> " + str(porta))
                    vetPortasLivres.remove(porta)
                    break
                except:
                    continue
                    
    def __setVelocidadeAndar(self, velocidade, aceleracao=None):
        inAtualizar = False
        if (velocidade!=self.__velocidadeAndarAtual):
            self.__velocidadeAndarAtual = velocidade
            inAtualizar = True
        if (aceleracao!=self.__aceleracaoAndarAtual):
            if (aceleracao==None):
                self.__aceleracaoAndarAtual=self.__aceleracaoPadraoAndar
            else:
                self.__aceleracaoAndarAtual=aceleracao
            inAtualizar = True
        if (inAtualizar):
            self.robot.brake()
            self.robot.settings(self.__velocidadeAndarAtual, self.__aceleracaoAndarAtual, self.__velocidadeGirarAtual, self.__aceleracaoGirarAtual)
            
    def __setVelocidadeGirar(self, velocidade,  aceleracao=None):
        inAtualizar = False
        if (velocidade!=self.__velocidadeGirarAtual):
            self.__velocidadeGirarAtual = velocidade
            inAtualizar = True
        
        if (aceleracao!=self.__aceleracaoGirarAtual):
            if (aceleracao==None):
                self.__aceleracaoGirarAtual=self.__aceleracaoPadraoAndar
            else:
                self.__aceleracaoGirarAtual=aceleracao
            inAtualizar = True

        if (inAtualizar):
            self.robot.brake()
            self.robot.settings(self.__velocidadeAndarAtual, self.__aceleracaoPadraoAndar, self.__velocidadeGirarAtual, self.__aceleracaoGirarAtual)
 
    def __ajusteAngulo(self, angulo):
        angulo = angulo%360
        if (angulo<0.0):
            angulo = 360+angulo
        
        return angulo
    
    def __leituraAngulo(self, ajustado=True, sensorGiro=False):
        #Testando para ver se apenas o self.hub.imu.heading() funciona
        sensorGiro=False
        if (sensorGiro):
            return self.__ajusteAngulo(self.robot.angle())
        else:
            return self.__ajusteAngulo(self.hub.imu.heading())
    
    def __reiniciarAngulo(self, anguloReiniciar=0):
        self.robot.reset()
        self.___anguloDefinido = anguloReiniciar
        self.hub.imu.reset_heading(anguloReiniciar)
        
    def getInclinacao(self):
        #Retorna o ângulo de inclinação
        #(passo, rolagem)
        # passo - positivo, hub inclinado para tras, negativo, hub inclinado para frente
        # rolagem - positivo, hub inclinado para esquerda, negativo, hub inclinado para direita
        vetor = self.hub.imu.tilt()
        passo = vetor[0]
        rolagem = vetor[1]
        return [passo,rolagem]  
            
    def statusBateria(self):
        if (self.hub.imu.ready()) and (self.hub.imu.stationary()):
            print("Carga da bateria: " + str(self.hub.battery.current()))
            if (self.hub.battery.current()<20):
                self.hub.light.on(Color.VIOLET)               
                self.hub.speaker.beep(200, 50)
                wait(50)
                self.hub.speaker.beep(400, 100)
                wait(100)
                self.hub.speaker.beep(500, 200)
            elif (self.hub.battery.current()<80):
                self.hub.light.on(Color.YELLOW)               
                self.hub.speaker.beep(100, 30)
            else:
                self.hub.light.on(Color.GREEN)                   
    
    def setLog(self, inLog):
        self.__inLog=inLog
    
    def validacoes(self, inDistancia=True, inGirar=True):
        if (inDistancia):
            self.validarDistancia()
        if (inGirar):
            self.validarGiro()
    
    def validarDistancia(self):        
        self.__inLog = True
        print("Andar 10 centimetros")
        self.andarParaFrente(100)
        print("Andar 30 centimetros")
        self.andarParaFrente(300)
        print("Andar 50 centimetros")
        self.andarParaFrente(500)
        print("Se andou menos do que a distância pretendida, deve dimunuir o xxx, caso contrário aumentar")
    
    def validarGiro(self):        
        self.__inLog = True
        print("Girar horário 90 graus")
        self.girarHorario(90)
    
    def getAnguloIdeal(self):
        anguloAtual = self.__leituraAngulo(False)
        return [self.anguloIdeal,anguloAtual]

    def getAnguloDefinido(self):
        return self.__anguloDefinido
    
    def setAnguloIdeal(self, angulo):
        self.anguloIdeal= angulo
    
    def ajustarAnguloIdeal(self, anguloDesejado=None, modo="NORMAL", tolerancia=0.2, maximoTentativas=8):
        
        if (anguloDesejado!=None):
            self.anguloIdeal = anguloDesejado
        
        tentativas = 0
        graus = 100

        while (tentativas<maximoTentativas) and (abs(graus)>tolerancia):
            anguloAtual = self.__leituraAngulo(False)
            graus = self.anguloIdeal - anguloAtual
            if (graus<-180):
                graus = graus + 360

            #print("{}-{}={}".format(str(self.anguloIdeal), str(anguloAtual), str(graus)))
            graus = graus
            if (abs(graus)>tolerancia):
                #print("Girar {} graus: ".format(str(graus)))
                #print(round(graus))
                self.__girar(round(graus), 200, modo, True, None, maximoTentativas, atualizarAnguloIdeal=False)
                #print("Depois do ajuste")
                #print(self.getAnguloIdeal())
            tentativas+=1
        #print("Terminou o ajuste")

    def limpar(self):
        self.robot.drive(100, 100)        
        while not pressed:
            continue
        return
        
    #------------------ INICIAR ----------------------------------    
    def iniciar(self, anguloPartida=0, posicaoTapete=[0,0], parametroAjusteGirar=1, velocidadePadraoAndar=None, velocidadePadraoGirar=None, inVerificacaoParado=False, inAjusteLancamento=False, inTratamentoErro=False, inLog=False):

        self.vetPosicaoTapete = posicaoTapete
        self.__anguloPartida = anguloPartida
        self.__parametroAjusteGirar=parametroAjusteGirar     
        self.__inTratamentoErro = inTratamentoErro
        self.__inLog = inLog

        self.__reiniciarAngulo(self.__anguloPartida)
        
        self.erroEixoX = 0
        self.erroEixoY = 0
        self.anguloIdeal = self.__anguloPartida
        
        if (inAjusteLancamento):
            self.ajustarAnguloIdeal(self.anguloIdeal)

        if (velocidadePadraoAndar!=None):
            self.__velocidadePadraoAndar = velocidadePadraoAndar
        
        if (velocidadePadraoGirar!=None):
            self.__velocidadePadraoGirar = velocidadePadraoGirar
        
        if (inVerificacaoParado):
            while True:
                if (self.hub.imu.ready()) and (self.hub.imu.stationary()):          
                    break
                else:
                    self.hub.light.on(Color.RED)
                    self.hub.speaker.beep(200, 100)
                    wait(500)
        
        if (inLog):
            print ("Robo iniciado com sucesso!")
            if (self.motorCentralPrincipal!=None):
                self.motorCentralPrincipal.setInLog(inLog)
            if (self.motorCentralAuxiliar!=None):
                self.motorCentralAuxiliar.setInLog(inLog)
            if (self.motorCentralConjunto!=None):
                self.motorCentralConjunto.setInLog(inLog)
            if (self.motorLateralDireito!=None):
                self.motorLateralDireito.setInLog(inLog)
            if (self.motorLateralEsquerdo!=None):
                self.motorLateralEsquerdo.setInLog(inLog)
            if (self.motorLateralConjunto!=None):
                self.motorLateralConjunto.setInLog(inLog)
            
    
    #------------------ TRATAMENTO DE ERRO ----------------------------------    
    def __zerarDesvios(self):
        self.erroEixoX = 0
        self.erroEixoY = 0
    
    def __parametros(self, sensorGiro=False, inLog=False):
        
        '''
        retorna um vetor
            [0] - distanciaPercorrida
            [1] - angulo Robo Ajustado
            [2] - angulo Robo Original - Leitura
            [3] - obstaculo, apenas tiver usando sensor de distância, caso contrário retorna 2000 (distância máxima)
        '''
        distanciaPercorrida = self.robot.distance()
        anguloRoboOriginal = self.hub.imu.heading()
        anguloRoboAjustado = self.__ajusteAngulo(anguloRoboOriginal)
        obstaculo = 255
        mensagem = "Dist {}, AngRoboAjustado {},  AngRoboOriginal {}".format(str(distanciaPercorrida), str(anguloRoboAjustado), str(anguloRoboOriginal))
        if(self.__ultraSonicSensor != None):
            obstaculo = self.leituraSensorDistancia()        
            mensagem = "{}, Obstáculo {}".format(mensagem, str(obstaculo))        
            
        if (inLog):
            print(str(mensagem)) 
        
        vetRetorno = [] 
        vetRetorno.append(distanciaPercorrida)
        vetRetorno.append(anguloRoboAjustado)
        vetRetorno.append(anguloRoboOriginal)
        vetRetorno.append(obstaculo)    

        return vetRetorno

    def __tratamentoErro(self, comando, parametro, sensorGiro=False):
    
        if (self.__inTratamentoErro==False):
            return

        vetRetorno = self.__parametros()

        if (comando=="ANDAR"):
            anguloDesconto = 0
            hipotenusa = parametro
            
            #Atualizar valor do tapete
            if (self.anguloIdeal==0):
                self.vetPosicaoTapete[1] = self.vetPosicaoTapete[1] + parametro
            elif (self.anguloIdeal==90):
                self.vetPosicaoTapete[0] = self.vetPosicaoTapete[0] + parametro
            elif (self.anguloIdeal==180):
                self.vetPosicaoTapete[1] = self.vetPosicaoTapete[1] - parametro
            elif (self.anguloIdeal==270):
                self.vetPosicaoTapete[0] = self.vetPosicaoTapete[0] - parametro
            else:
                if (self.anguloIdeal>0) and (self.anguloIdeal<90):            
                    anguloDesconto = 90
                elif (self.anguloIdeal>90) and (self.anguloIdeal<180):            
                    anguloDesconto = 180
                elif (self.anguloIdeal>180) and (self.anguloIdeal<270):            
                    anguloDesconto = 270
                elif (self.anguloIdeal>270) and (self.anguloIdeal<360):            
                    anguloDesconto = 360
                
                #Calcular o ideal
                anguloCalculoIdeal = anguloDesconto-self.anguloIdeal
                senoIdeal = umath.sin(umath.radians(anguloCalculoIdeal))
                cossenoIdeal = umath.cos(umath.radians(anguloCalculoIdeal))
                catetoOpostoIdeal = senoIdeal*hipotenusa
                catetoAdjacenteIdeal = cossenoIdeal*hipotenusa

                if (self.anguloIdeal>0) and (self.anguloIdeal<90):            
                    self.vetPosicaoTapete[0] = self.vetPosicaoTapete[0] +  catetoAdjacenteIdeal
                    self.vetPosicaoTapete[1] = self.vetPosicaoTapete[1] +  catetoOpostoIdeal                            
                elif (self.anguloIdeal>90) and (self.anguloIdeal<180):            
                    self.vetPosicaoTapete[0] = self.vetPosicaoTapete[0] +  catetoOpostoIdeal
                    self.vetPosicaoTapete[1] = self.vetPosicaoTapete[1] -  catetoAdjacenteIdeal
                elif (self.anguloIdeal>180) and (self.anguloIdeal<270):            
                    self.vetPosicaoTapete[0] = self.vetPosicaoTapete[0] -  catetoAdjacenteIdeal
                    self.vetPosicaoTapete[1] = self.vetPosicaoTapete[1] -  catetoOpostoIdeal
                elif (self.anguloIdeal>270) and (self.anguloIdeal<360):            
                    self.vetPosicaoTapete[0] = self.vetPosicaoTapete[0] -  catetoOpostoIdeal
                    self.vetPosicaoTapete[1] = self.vetPosicaoTapete[1] +  catetoAdjacenteIdeal
 
            #Tratamento de erro
            anguloGyroSensor = vetRetorno[1]
            desvioX = 0
            desvioY = 0
            if (anguloGyroSensor!=None) and (anguloGyroSensor!=self.anguloIdeal):
                desvioAngulo = 0
                if (self.anguloIdeal==0):
                    desvioAngulo = vetRetorno[2]
                    if (desvioAngulo>=360):
                        desvioAngulo = vetRetorno[2]%360
                    elif (desvioAngulo<=-360):
                        desvioAngulo = vetRetorno[2]%-360
                    
                    seno = umath.sin(umath.radians(90-abs(desvioAngulo)))
                    cosseno = umath.cos(umath.radians(abs(desvioAngulo)))
                    desvioX = hipotenusa - hipotenusa*seno
                    print(desvioX)
                    if (desvioAngulo<0):
                        desvioX = desvioX*-1    
                    desvioY = hipotenusa-(hipotenusa*cosseno) #Sempre positivo     
                    
                elif (self.anguloIdeal==90):
                    desvioAngulo = self.anguloIdeal-anguloGyroSensor
                    angulo = abs(desvioAngulo)
                    rad = umath.radians(angulo)
                    seno = umath.sin(rad)
                    cosseno = umath.cos(rad)
                    desvioX = hipotenusa-(hipotenusa*cosseno) #Sempre positivo    
                    desvioY = (hipotenusa*seno)
                    #auxDesvio = umath.sqrt(hipotenusa*hipotenusa + (hipotenusa+desvioX)*(hipotenusa+desvioX))
                    #print(auxDesvio)
                    #desvioY = umath.sqrt(hipotenusa*hipotenusa + (hipotenusa-desvioX)*(hipotenusa-desvioX))
                    if (desvioAngulo<0):
                        desvioY = desvioY*-1

                elif (self.anguloIdeal==180):
                    desvioAngulo = self.anguloIdeal-anguloGyroSensor
                    angulo = abs(desvioAngulo)
                    rad = umath.radians(angulo)
                    seno = umath.sin(rad)
                    cosseno = umath.cos(rad)
                    
                    desvioX = hipotenusa*seno
                    if (desvioAngulo<0):
                        desvioX = desvioX*-1
                    
                    desvioY = (hipotenusa*cosseno) - hipotenusa #Sempre negativo

                elif (self.anguloIdeal==270):
                    desvioAngulo = self.anguloIdeal-anguloGyroSensor
                    angulo = abs(desvioAngulo)
                    rad = umath.radians(angulo)
                    seno = umath.sin(rad)
                    cosseno = umath.cos(rad)
                    desvioX = (hipotenusa*cosseno)-hipotenusa #Sempre negativo
    
                    desvioY = hipotenusa*seno     
                    if (desvioAngulo<0):
                        desvioY = desvioY*-1
                    
                else:
                    anguloCalculoExecutado = anguloDesconto-anguloGyroSensor

                    #Calcular o executado
                    senoExecutado = umath.sin(umath.radians(anguloCalculoExecutado))
                    cossenoExecutado = umath.cos(umath.radians(anguloCalculoExecutado))
                    catetoOpostoExecutado = senoExecutado*parametro
                    catetoAdjacenteExecutado = cossenoExecutado*parametro
    
                    if (self.anguloIdeal>0) and (self.anguloIdeal<90):            
                        desvioX = catetoAdjacenteExecutado - catetoAdjacenteIdeal
                        desvioY = catetoOpostoExecutado - catetoOpostoIdeal                            
                    elif (self.anguloIdeal>90) and (self.anguloIdeal<180):            
                        desvioX = catetoOpostoExecutado - catetoOpostoIdeal
                        desvioY = catetoAdjacenteExecutado - catetoAdjacenteIdeal
                    elif (self.anguloIdeal>180) and (self.anguloIdeal<270):            
                        desvioX = catetoAdjacenteIdeal-catetoAdjacenteExecutado
                        desvioY = catetoOpostoIdeal-catetoOpostoExecutado
                    elif (self.anguloIdeal>270) and (self.anguloIdeal<360):            
                        desvioX = catetoOpostoIdeal-catetoOpostoExecutado
                        desvioY = catetoAdjacenteIdeal-catetoAdjacenteExecutado
        
                desvioX = round(desvioX, 2)
                desvioY = round(desvioY, 2)

                self.erroEixoX += desvioX
                self.erroEixoY += desvioY
                
                self.erroEixoX = round(self.erroEixoX, 2)
                self.erroEixoY = round(self.erroEixoY, 2)
                
                if self.__inLog:
                    print(self.vetPosicaoTapete)
                    print("Distância {}, Ideal {}, Leitura {}, Desvio Angulo, {}".format(str(parametro), str(self.anguloIdeal), str(anguloGyroSensor), str(desvioAngulo)))
                    print("DesvioX {}, DesvioY {}, Erro Acumulado X {}, Erro Acumulado Y {}".format(str(desvioX), str(desvioY), str(self.erroEixoX), str(self.erroEixoY)))                
        else:
            if self.__inLog:
                print("Sem desvio")
                
            
        if (comando.find("GIRAR")!=-1):
            print(self.anguloIdeal)
            print(self.__leituraAngulo())
        '''
        if (comando=="GARRA"):
            print("Não implementado")
        '''
    
    def imprimirPosicao(self):
        print(self.vetPosicaoTapete)
        print(self.anguloIdeal)
        print(self.erroEixoX)
        print(self.erroEixoY)
        
        
    #------------------ANDAR ----------------------------------    
    def andarParaFrente(self, distancia, velocidade=None, distanciaMaxima=None, sensorGiro=True, ajusteErro=False, riscoStalled=False, tratamentoStalled=[None,None,None,None], wait=True, stop=True):
        self.__andar(distancia, velocidade, distanciaMaxima, sensorGiro, ajusteErro, riscoStalled, tratamentoStalled, wait, stop)      
    
    def andarParaTras(self, distancia, velocidade=None, distanciaMaxima=None, sensorGiro=True, ajusteErro=False, riscoStalled=False, tratamentoStalled=[None,None,None,None], wait=True, stop=True):
        self.__andar(-distancia, velocidade, distanciaMaxima, sensorGiro, ajusteErro, riscoStalled, tratamentoStalled, wait, stop)  
   
    def andarAtePonto(self, pontoX=0, pontoY=0, velocidade=None, sensorGiro=False, ajusteErro=False, wait=True, stop=True):
        posicaoAtualX = self.vetPosicaoTapete[0]
        distanciaX =  pontoX - posicaoAtualX
        self.__andar(distanciaX, velocidade, False, ajusteErro, inLog)

    def __andar(self, distancia, velocidade, distanciaMaxima, sensorGiro, ajusteErro, riscoStalled, tratamentoStalled, wait, stop):
        
        if (velocidade!=None) and (velocidade!=self.__velocidadeAndarAtual):
            self.__setVelocidadeAndar(velocidade)        
        elif (velocidade==None) and (self.__velocidadePadraoAndar!=self.__velocidadeAndarAtual):
            self.__setVelocidadeAndar(self.__velocidadePadraoAndar) 
        
        if (self.__inTratamentoErro==False):
            ajusteErro = False

        self.robot.reset()
        self.robot.use_gyro(sensorGiro)
        
        distanciaAjustada = distancia
        if (ajusteErro):
            if (self.__inLog):
                print("Erro X {}, erro y {} ".format(str(self.erroEixoX), str(self.erroEixoY)))
            
            if (self.anguloIdeal==0) and (self.erroEixoY!=0):
                distanciaAjustada = distancia - self.erroEixoY
                self.erroEixoY = 0
            elif (self.anguloIdeal==90) and (self.erroEixoX!=0):
                distanciaAjustada = distancia - self.erroEixoX
                self.erroEixoX = 0
            elif (self.anguloIdeal==180) and (self.erroEixoY!=0):
                distanciaAjustada = distancia + self.erroEixoY
                self.erroEixoY = 0
            elif (self.anguloIdeal==270) and (self.erroEixoX!=0):
                distanciaAjustada = distancia + self.erroEixoX
                self.erroEixoX = 0
        
            if (self.__inLog):
                print("Log Andar: Ângulo ideal={}, distância desejada={}, distância ajustada={}".format(str(self.anguloIdeal), str(distancia), str(distanciaAjustada)))
        
        if (wait==False):
            self.robot.straight(distancia, then=Stop.HOLD, wait=False)        
        else:
            if (distanciaMaxima!=None):
                while(True):
                    self.robot.straight(distancia, then=Stop.HOLD, wait=False)
                    if (abs(self.robot.distance())>distanciaMaxima):
                        break
            else:
                if (riscoStalled==False):
                    self.robot.straight(distanciaAjustada)
                    if (stop):
                        self.robot.stop()
                else:
                    stopwatch = StopWatch()

                    stalled=False
                    self.robot.straight(distancia, then=Stop.HOLD, wait=False)
                    distanciaAndada = 0
                    tempoAndar = stopwatch.time()
                    while(self.robot.done()==False):
                        wait(10)
                        distancia = self.robot.distance()
                        if (distanciaAndada==distancia):
                            if (stopwatch.time()==tempoAndar+2000):
                                print("Stallled")
                        else:
                            distanciaAndada = distancia 
                            tempoAndar = stopwatch.time()
                
    
        self.__tratamentoErro("ANDAR", distancia, sensorGiro)      
 
    #------------------GIRAR ----------------------------------  
    def girarHorario(self, graus, velocidade=None, modo="DRIVE", sensorGiro=True, parametroGirar=None, maximoTentativas=5):
        self.__girar(graus, velocidade, modo, sensorGiro, parametroGirar, maximoTentativas)
        
    def girarAntiHorario(self, graus, velocidade=None, modo="DRIVE", sensorGiro=True, parametroGirar=None, maximoTentativas=5):
        self.__girar(-graus, velocidade, modo, sensorGiro, parametroGirar, maximoTentativas)
    
    def __calcularAnguloMovimentar(self, sentido, inicial, final):
        ret = 0
        inicial = float(inicial)
        final = float(final)
        if (round(inicial, 3)==round(final, 3)):
            return ret

        if (sentido==1):
            #movimento com graus crescente
            ret = final - inicial
            if (inicial>final):
                ret = ret + 360

        else:
            #movimento com graus decrescente
            ret = inicial-final
            if (final>inicial):
                ret = ret + 360
        
        return ret%360
    
    def __girar(self, graus, velocidade, modo, sensorGiro, parametroGirar, maximoTentativas, atualizarAnguloIdeal=True):        
        '''
        Métodos de Girargraus:
        5 tipos
        "DRIVE":            utiliza o método turn da classe DriveBase
        "MOTORDIREITO":     move apenas o motor direito e o esquerdo fica parado.
        "MOTORESQUERDO":    move apenas o motor esquerdo e o direito fica parado.
        "DOISMOTORES":      sentido horário, motor direito vai pra tras e esquerdo para frente
                            sentido anti-horário, motor direito vai pra frente e esquerdo para tras
        "NORMAL":           girar mantendo o centro do robo sem modificação                        
        '''
        if (graus==0):
            return

        if (velocidade==None):
            velocidade = self.__velocidadePadraoGirar

        if (parametroGirar==None):
            parametroGirar = self.__parametroAjusteGirar

        self.robot.use_gyro(sensorGiro)

        if (atualizarAnguloIdeal):
            self.anguloIdeal = self.__ajusteAngulo(self.anguloIdeal + graus)

        if (modo=="DRIVE"):
            if (velocidade!=None) and (velocidade!=self.__velocidadeGirarAtual):
                self.__setVelocidadeGirar(velocidade)      
            elif (velocidade==None) and (self.__velocidadePadraoGirar!=self.__velocidadeGirarAtual):
                self.__setVelocidadeGirar(self.__velocidadePadraoGirar)

            self.__girarDrive(graus, sensorGiro)
        elif (modo=="MOTORDIREITO"):
            sentidoMotor = -1
            if (graus<0):
                sentidoMotor = 1
            self.__girarMotor(self.__motorRodaDireita, graus, velocidade, sentidoMotor, sensorGiro, parametroGirar, maximoTentativas)                
        elif (modo=="MOTORESQUERDO"):
            sentidoMotor = 1
            if (graus<0):
                sentidoMotor = -1
            self.__girarMotor(self.__motorRodaEsquerda, graus, velocidade, sentidoMotor, sensorGiro, parametroGirar, maximoTentativas)                
        elif (modo=="DOISMOTORES"):
            self.__girarDoisMotores(-graus, velocidade, sensorGiro, parametroGirar, maximoTentativas)
        else:
            self.__andarCurvo(0, graus, velocidade)
            
        if (self.__inLog):
            print("Log Girar: Angulo ideal={}, Final Hub={}".format(str(self.anguloIdeal), str(self.__leituraAngulo(False))))
        
        
        #self.__tratamentoErro("GIRAR" + "|"  + modo, graus, sensorGiro) 
    
    def __girarDrive(self, graus, sensorGiro, variacaoPermitida=0.10, maximoTentativas=5, inLog=True):
        
        '''
        self.robot.reset()
        self.robot.use_gyro(True)
        self.robot.turn(graus)
        self.robot.stop()       
        
        if (inLog):
            print("Hub: " + str(self.__leituraAngulo(False)))
            
        return
        '''

        hub = self.__leituraAngulo(False)
        self.robot.turn(graus*self.__parametroAjusteGirar)
        hub = self.__leituraAngulo(False)
        variacao = self.anguloIdeal - hub               
        while(abs(variacao)>=abs(variacaoPermitida)):
            self.robot.turn(variacao)
            hub = self.__leituraAngulo(False)
            variacao = self.anguloIdeal - hub   
            if (inLog):
                print("Hub: " + str(hub) + " Variação: " + str(variacao))
            
        #while(>abs(self.robot.angle())):

        self.robot.stop()       
                         
    def __girarMotor(self, motorGirar, graus, velocidade, sentidoMotor, sensorGiro, parametroGirar, maximoTentativas, margemErro=0.005):                
        
        self.robot.stop()
        self.robot.reset()
        
        if (parametroGirar==None):
            parametroGirar = 2


        sentidoGirar = 1
        if (graus<0):
            sentidoGirar = -1

        anguloInicial = self.__leituraAngulo(sensorGiro)
        anguloMovimentar = self.__calcularAnguloMovimentar(sentidoGirar, self.hub.imu.heading(), self.anguloIdeal)
        if (self.__inLog):
            print ("Inicial {}, objetivo  {}, movimentar {}".format(str(anguloInicial), str(self.anguloIdeal), str(anguloMovimentar)))
        
        anguloRestante = anguloMovimentar
        tentativa = 0
        while ((abs(anguloRestante)>0) and (anguloMovimentar>=anguloRestante) and (anguloRestante>=margemErro) and (tentativa<maximoTentativas)):
            motorGirar.run_angle(velocidade, sentidoMotor*anguloRestante*umath.pi*parametroGirar, then=Stop.COAST_SMART, wait=True)
            anguloRestante = self.__calcularAnguloMovimentar(sentidoGirar, self.hub.imu.heading(),self.anguloIdeal) 
            tentativa=tentativa+1
        
        if (self.__inLog):
            resultado = self.__leituraAngulo(sensorGiro)
            print ("Final: {}".format(str(resultado)))
        
        motorGirar.stop()
        
    def __girarDoisMotores(self, graus, sensorGiro, parametroGirar, maximoTentativas, margemErro=0.005):    
        
        if (parametroGirar==None):
            parametroGirar = 1
        velocidade = 300
        self.robot.stop() 
        self.robot.reset()

        #sentido invertifo mesmo
        sentidoGirar = -1
        if (graus<0):
            sentidoGirar = 1
        
        anguloInicial = self.__leituraAngulo(sensorGiro)
        anguloMovimentar = self.__calcularAnguloMovimentar(sentidoGirar, anguloInicial, self.anguloIdeal)
        print(str(anguloMovimentar) + "/" + str(velocidade))
        anguloRestante = anguloMovimentar
        tentativa = 0
        primeiroComando = True
        while (anguloMovimentar>=anguloRestante) and (anguloRestante>=margemErro) and (abs(anguloRestante)>0):
            print("movimentar {}/ restante {}".format(str(anguloMovimentar), str(anguloRestante)))
            if (self.__motorRodaDireita.done()) or (primeiroComando):
                print(anguloRestante)
                primeiroComando=False
                self.__motorRodaDireita.run_angle(velocidade, graus, then=Stop.COAST_SMART, wait=False)
                self.__motorRodaEsquerda.run_angle(velocidade,-graus, then=Stop.COAST_SMART, wait=False)
            anguloRestante = self.__calcularAnguloMovimentar(sentidoGirar, self.hub.imu.heading(), self.anguloIdeal) 

        self.__motorRodaDireita.stop()
        self.__motorRodaEsquerda.stop()
        
    #------------------ANDAR CURVO----------------------------------  
    def andarCurvoHorarioFrente(self, radiano, angulo, velocidade=None):
        self.__andarCurvo(radiano, angulo, velocidade) 
        self.anguloIdeal = self.__ajusteAngulo(self.anguloIdeal + angulo)

    def andarCurvoHorarioTras(self, radiano, angulo, velocidade=None):
        self.__andarCurvo(-radiano, -angulo, velocidade) 
        self.anguloIdeal = self.__ajusteAngulo(self.anguloIdeal - angulo)

    def andarCurvoAntiHorarioFrente(self, radiano, angulo, velocidade=None):
        self.__andarCurvo(radiano, -angulo, velocidade) 
        self.anguloIdeal = self.__ajusteAngulo(self.anguloIdeal - angulo)
    
    def andarCurvoAntiHorarioTras(self, radiano, angulo, velocidade=None):
        self.__andarCurvo(-radiano, angulo, velocidade) 
        self.anguloIdeal = self.__ajusteAngulo(self.anguloIdeal + angulo)

    def __andarCurvo(self, radiano, angulo, velocidade):
        if (velocidade!=None) and (velocidade!=self.__velocidadeAndarAtual):
            self.__setVelocidadeAndar(velocidade)        
        elif (velocidade==None) and (self.__velocidadePadraoAndar!=self.__velocidadeAndarAtual):
            self.__setVelocidadeAndar(self.__velocidadePadraoAndar) 
        
        self.robot.curve(radiano, angulo) 
        self.robot.stop()

    #------------------MOTORES INTERFACE ----------------------------------  
    
    '''
    def moverPrincipalParaAngulo(self, anguloDesejado, velocidade=None, wait=True, rangeErro=0, riscoStalled=False, tratamentoStalled=[None,None,None,None]):
        
        if (anguloDesejado==self.__anguloMotorPrincipal):
            return
        
        if (self.__anguloMotorPrincipal!=None):
            if (anguloDesejado<0):
                riscoStalled = True
                tratamentoStalled=[None,None,None,None]
            elif (anguloDesejado>self.__anguloMaximoMotorPrincipal):
                anguloDesejado = self.__anguloMaximoMotorPrincipal
        
        if (velocidade==None):
                velocidade = self.__velocidadePadraoMotorPrincipal

        self.__moverMotorAngulo(self.__motorPrincipal, self.__parametroAnguloMotorPrincipal, self.__anguloMotorPrincipal, anguloDesejado, velocidade, rangeErro, riscoStalled, tratamentoStalled)
        self.__anguloMotorPrincipal = anguloDesejado
        if (self.__anguloMotorPrincipal<0):
            self.__anguloMotorPrincipal=0
            self.__motorPrincipal.reset_angle(0)
        
    def moverAuxiliarParaAngulo(self, anguloDesejado, velocidade=None, wait=True, rangeErro=0, riscoStalled=False, tratamentoStalled=[None,None,None,None]):
        
        if (anguloDesejado==self.__anguloMotorAuxiliar):
            return
        
        if (self.__anguloMaximoMotorAuxiliar!=None):
            if (anguloDesejado<0):
                anguloDesejado = 0
            elif (anguloDesejado>self.__anguloMaximoMotorAuxiliar):
                anguloDesejado = self.__anguloMaximoMotorAuxiliar

        if (velocidade==None):
                velocidade = self.__velocidadePadraoMotorAuxiliar

        self.__moverMotorAngulo(self.__motorAuxiliar, self.__parametroAnguloMotorAuxiliar, self.__anguloMotorAuxiliar, anguloDesejado, velocidade, wait, rangeErro, riscoStalled, tratamentoStalled)
        self.__anguloMotorAuxiliar = anguloDesejado
            
    def abrirGarraPrincipal(self, levantado=False, wait=True):
        
        #anguloAbertura = 250 + self.__anguloMotorPrincipal
        #self.__moverMotorParaPosicaoComandoAngle(self.__motorPrincipal, -anguloAbertura, velocidadeDesejada=800, wait=wait)
        #self.__anguloMotorPrincipal = 0
        
        anguloMover = 160
        if (levantado):
            anguloMover += 40

        self.__moverMotorParaPosicaoComandoAngle(self.__motorPrincipal, anguloMover, velocidade=800, wait=wait)
        
    def fecharGarraPrincipal(self, levantar=False, wait=True):
        anguloMover = 150
        if (levantar):
            anguloMover += 40
        self.__moverMotorParaPosicaoComandoAngle(self.__motorPrincipal, -anguloMover, velocidade=800, wait=wait)

    def levantarGarraPrincipal(self, angulo=40, wait=True):    
        self.__moverMotorParaPosicaoComandoAngle(self.__motorPrincipal, -angulo, velocidade=800, wait=wait)

    def abrirGarraAuxiliar(self, levantado=False, wait=True):
        anguloMover = 200
        if (levantado):
            anguloMover += 70
        self.__moverMotorParaPosicaoComandoAngle(self.__motorAuxiliar, anguloMover, velocidade=800, wait=wait)
         
    def fecharGarraAuxiliar(self, levantar=False, wait=True):
        anguloMover = 200+10
        if (levantar):
            anguloMover += 70
            
        self.__moverMotorParaPosicaoComandoAngle(self.__motorAuxiliar, -anguloMover, velocidade=800, wait=wait)
             
    def levantarGarraAuxiliar(self, angulo=70, wait=True):    
        self.__moverMotorParaPosicaoComandoAngle(self.__motorAuxiliar, -angulo, velocidade=800, wait=wait)
    
    '''
    def aguardarTerminoMovimento(self, centralPrincipal=True, centralAuxiliar=True, lateralDireito = True, lateralEsquerdo = True):

        if (self.motorCentralPrincipal==None):
            centralPrincipal = False
        if (self.motorCentralAuxiliar==None):
            centralAuxiliar = False
        if (self.motorLateralDireito==None):
            lateralDireito = False
        if (self.motorLateralEsquerdo==None):
            lateralEsquerdo= False
        
        if (centralPrincipal) and (centralAuxiliar) and (lateralDireito)  and (lateralEsquerdo):
            while (self.motorCentralPrincipal.done()==False) and (self.motorCentralAuxiliar.done()==False) and (self.motorLateralDireito.done()==False) and (self.motorLateralEsquerdo.done()==False):
                wait(10)
            return
        elif (lateralDireito) and (lateralEsquerdo):
            while(self.motorLateralDireito.done()==False) and (self.motorLateralEsquerdo.done()==False):
                wait(10)
            return
        elif (centralPrincipal) and (centralAuxiliar):
            while(self.motorCentralPrincipal.done()==False) and (self.motorCentralAuxiliar.done()==False):
                wait(10)
            return
        elif centralPrincipal:
            while(self.motorCentralPrincipal.done()==False):
                wait(10)
        elif centralAuxiliar:
            while(motorCentralAuxiliar.done()==False):
                wait(10)
        elif lateralDireito:
            while(motorLateralDireito.done()==False):
                wait(10)
        elif lateralEsquerdo:
            while(motorLateralEsquerdo.done()==False):
                wait(10)
        else:
            return
        
    
    #------------------PROGRAMAS   ----------------------------------  
    
    def __moverMotorAngulo(self, motorMovimento, parametroMotor, anguloAtualMotor, anguloDesejado, velocidade=None, wait=True, rangeErro=0, riscoStalled=False, tratamentoStalled=[None,None,None,None]):
        '''
        tratamentoStalled=['Andar para Frente','Andar para Tras','Girar Horário','Girar Anti Horário']
        '''
        
        #anguloMovimentar = (anguloDesejado - anguloAtualMotor)*parametroMotor
        anguloMovimentar = (anguloDesejado - (motorMovimento.angle()/parametroMotor))*parametroMotor 

        if (self.__inLog):
            print("Angulo atual motor {}, angulo real do motor {}".format(str(anguloAtualMotor), str(motorMovimento.angle())))
            print ("Log Garra: Ângulo atual={}/ Ângulo desejado={}/ Ângulo movimentar={}".format(str(anguloAtualMotor), str(anguloDesejado), str(anguloMovimentar)))
        
        terminoMovimento = self.__moverMotorParaPosicaoComandoAngle(motorMovimento, anguloMovimentar, velocidade, wait, riscoStalled)

        if (terminoMovimento==False):
            if (self.__inLog):
                print("Stalled " + str(motorMovimento.angle()))
            
            if (motorMovimento.stalled()):
                if (tratamentoStalled[0]==None) and (tratamentoStalled[1]==None) and (tratamentoStalled[2]==None) and (tratamentoStalled[3]==None):
                    motorMovimento.stop()
                    if (self.__inLog):
                        print("Sem tratamento de erro ")
                    return
                
                if (tratamentoStalled[0]) or (tratamentoStalled[1]):
                    self.__setVelocidadeAndar(40)                
                if (tratamentoStalled[2]) or (tratamentoStalled[3]):
                    self.__setVelocidadeGirar(40)                
                    
                while(motorMovimento.done()==False):
                    primeiroComando = True
                    while(motorMovimento.stalled()):
                        if (tratamentoStalled[0]) or (tratamentoStalled[1]):
                            if (self.robot.done() or primeiroComando):
                                if (tratamentoStalled[2]==False) and (tratamentoStalled[3]==False):
                                    primeiroComando=False
                                    if (tratamentoStalled[0]):
                                        self.robot.straight(2, then=Stop.COAST_SMART, wait=False)
                                    else:
                                        self.robot.straight(-2, then=Stop.COAST_SMART, wait=False)
                                elif (tratamentoStalled[2]):
                                    #girar Sentido Horario
                                    primeiroComando=False
                                    if (tratamentoStalled[0]):
                                        #girar sentido horário para frente
                                        self.robot.curve(1, -1, then=Stop.COAST_SMART, wait=False)
                                    else:
                                        #girar sentido horário para tras
                                        self.robot.curve(1, 1, then=Stop.COAST_SMART, wait=False)

                                elif (tratamentoStalled[3]):
                                    #girar Sentido AntiHorario
                                    primeiroComando=False
                                    if (tratamentoStalled[0]):
                                        #girar sentido antihorário para frente
                                        self.robot.curve(-1, 1, then=Stop.COAST_SMART, wait=False)
                                    else:
                                        #girar sentido antihorário para tras
                                        self.robot.curve(-1, -1, then=Stop.COAST_SMART, wait=False)

                        else:
                            if (self.robot.done() or primeiroComando):
                                if (tratamentoStalled[2]):
                                    self.robot.turn(1, then=Stop.COAST_SMART, wait=False)
                                if (tratamentoStalled[3]):
                                    self.robot.turn(-1, then=Stop.COAST_SMART, wait=False)
                                primeiroComando=False
                                                        
                    self.robot.stop()
                
                motorMovimento.stop()

    def __moverMotorParaPosicaoComandoAngle(self, motorMovimento, anguloMovimentar, velocidade, wait=True, riscoStalled=False):
        
        motorMovimento.run_angle(velocidade, anguloMovimentar, wait=False)
        if (riscoStalled):
            wait=True
        if (wait):
            while(motorMovimento.stalled()==False):
                if (motorMovimento.done()): 
                    motorMovimento.stop()    
                    return True
            if (riscoStalled):
                return False
            else:
                return True
        else:
            return True


    #------------------SENSORES ---------------------------------- 
    def leituraSensorDistancia(self):
        #Retorna distÂncia em milimetros, maior distância identificada 2000 mm
        distancia = None
        if (self.__ultraSonicSensor!=None):
            return self.__ultraSonicSensor.distance()
        return distancia
            
    def leituraSensorForca(self):
        #Retorna  4 parâmetros
        pressionado = None  # True ou False - pode configurar a força mínima que deve ser considerado como pressionado, padrão é 3 
        tocado = None       # True ou False - mais sensível que o pressionado 
        forca = None        # Float - força da pressão sobre o botão
        distancia = None    # Float - distância em milimetro que o botão de movimentou, quando não ha toque ele fica como 0.047 ou 0.071                
        if (self.__forceSensor!=None):
            pressionado = self.__forceSensor.pressed()
            tocado = self.__forceSensor.touched()
            forca = self.__forceSensor.force()
            distancia = self.__forceSensor.distance()

        return pressionado, tocado, forca, distancia
    
    def leituraSensorCorPadrao(self, coresPossiveis=None):
        #retorna três parâmetros
        cor = None 
        luzRefletida  = None #0 - sem reflexão, 100 - muita luz refletida
        luzAmbiente =  None  #0 - escuro, 100 - muito brilho
        '''
        O sensor detecta as cores das peças LEGO
        -1 = No object 
        0 = Black (LEGO:26; R:0, G:0, B:0)
        1 = Magenta (LEGO:124; R:144, G:31, B:118)
        3 = Blue (LEGO:23; R:30, G:90, B:168)
        4 = Turquoise (LEGO:322; R:104, G:195, B226)
        5 = Green (LEGO:28; R:0, G:133, B:43)
        7 = Yellow (LEGO:24; R:250, G:200, B:10)
        9 = Red (LEGO:21; R:180, G:0, B:0)
        10 = White (LEGO:01; R:244, G:244, B:244)
        '''
        if (self.__colorSensor1!=None):
            if (coresPossiveis!=None):
                self.__colorSensor1.detectable_colors(coresPossiveis)
            cor = self.__colorSensor1.color()
            luzRefletida = self.__colorSensor1.reflection()
            luzAmbiente = self.__colorSensor1.ambient()
        return cor, luzRefletida, luzAmbiente
    
    def leituraSensorCor(self):
        #retorna cinco parâmetros
        matiz = None
        saturacao = None
        brilho = None
        luzRefletida  = None 
        luzAmbiente = None
        
        if (self.__colorSensor1!=None):
            vetHsv = self.__colorSensor1.hsv()
            matiz = vetHsv[0]
            saturacao = vetHsv[1]
            brilho = vetHsv[2]    
            luzRefletida = self.__colorSensor1.reflection()
            luzAmbiente = self.__colorSensor1.ambient()
        return matiz, saturacao, brilho, luzRefletida, luzAmbiente
    
    '''
    Método interessante cor
    wait_for_color(Color.RED)
    '''
    #------------------SEGUIR PELA LINHA ------------------------- 
    
    def seguirLinha(self, corLinha, portaSensorEsquerda, portaSensorDireita):
        return
    
    #------------------SUBIR RAMPA ------------------------- 
    def subirRampa(self, inclinacao, distancia, velocidade):
        self.robot.brake()
        self.robot.settings(straight_speed=150, straight_acceleration=100)

        while(True):
            self.robot.straight(distancia, then=Stop.HOLD, wait=False)
            print(self.getInclinacao())
            if (abs(self.robot.distance())>distancia):    
                break

   