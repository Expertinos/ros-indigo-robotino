#!/usr/bin/env python

import roslib; roslib.load_manifest('cbr2015_module_a_planner')
import rospy
import smach
import smach_ros
import sys
from Casa import casa
from LigarNavigation import ligarNavigation
from LigarNavigation import atualizaCmdVel
from BuscarPedido import buscarPedido
from BuscarProduto import buscarProduto
from VerificarProduto import verificarProduto
from PegarProduto import pegarProduto
from EntregarProduto import entregarProduto
from IrParaCasa import irParaCasa
from LigarLed import ligarLed
from PiscarLed import piscarLed
from std_msgs.msg import String
from enum import *
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
import time

global area_casa
global areas_depositos
global store_areas_depositos
global area_pedido
global areas_produtos
global area_numero
produtos = [Product.TV, Product.DVD, Product.CELULAR, Product.TABLET, Product.NOTEBOOK]
global pedidos #= [Product.DVD, Product.CELULAR, Product.TABLET]
deposito_flag = False
seq = 0

# define state Foo
class Casa(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['iniciar', 'termina'])
        self.counter = 0

    def execute(self, userdata):
	global new_order
        global termina
	rospy.logwarn("Esperando pedido")
	while new_order == False:
		#rospy.logwarn("while")
		if termina == True:
			return 'termina'
	rospy.logwarn("passou o while" + str(new_order))
	new_order = False
        casa()
        return 'iniciar'

# define state Bar
class LigarNavigation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['coleta_pedido', 'coleta_produto', 'voltar_casa', 'indo_deposito'],
						input_keys=['produto'])
        self.count = 0

    def execute(self, userdata):
	global deposito_flag
	global seq
	global areas_produtos
	global pedidos
	global area_numero

	seq += 1
	if(len(areas_produtos) == area_numero):
		rospy.logwarn("dentro do if - area_numero = "+str(area_numero))
		area_numero = 0
		del pedidos[0]
		userdata.produto = pedidos[0]

	if deposito_flag == True:
		rospy.logwarn("o produto carregado e "+str(userdata.produto))
		rospy.logwarn("deposito 1 "+str(Areas.STORE_DEPOSITO1[3]))
		rospy.logwarn("deposito 2 "+str(Areas.STORE_DEPOSITO2[3]))
		rospy.logwarn("deposito 3 "+str(Areas.STORE_DEPOSITO3[3]))

		if(userdata.produto in Areas.STORE_DEPOSITO1[3]):
			rospy.logwarn("deposito1")
			ligarNavigation(Areas.STORE_DEPOSITO1, seq, "Deposito1")

		if(userdata.produto in Areas.STORE_DEPOSITO2[3]):
			rospy.logwarn("deposito2")
			ligarNavigation(Areas.STORE_DEPOSITO2, seq, "Deposito2")

		if(userdata.produto in Areas.STORE_DEPOSITO3[3]):
			rospy.logwarn("deposito3")
			ligarNavigation(Areas.STORE_DEPOSITO3, seq, "Deposito3")


		deposito_flag = False
		return 'indo_deposito'

        if self.count == 0:
            self.count += 1
    	    ligarNavigation(Areas.PEDIDOS, seq, "Pedidos")
	    return 'coleta_pedido'

	if len(areas_produtos) == 0 or len(pedidos) == 0:
		ligarNavigation(Areas.CASA, seq, "Casa")
		return 'voltar_casa'

	rospy.logwarn("area_numero = "+str(area_numero))
	rospy.logwarn(areas_produtos[area_numero])
        ligarNavigation(areas_produtos[area_numero], seq, "Area"+str(area_numero))
        return 'coleta_produto'

class BuscarPedido(smach.State):
    def __init__(self):
	smach.State.__init__(self, outcomes=['peguei_pedido', 'voltar_casa'],
				input_keys=['produto'],
                                output_keys=['produto'])

    def execute(self, userdata):
	global pub
	global pedidos

	pedidos = buscarPedido(pub)
	if len(pedidos) == 0:
		ligarNavigation(Areas.CASA, seq, "Casa")
		return 'voltar_casa'

	userdata.produto = pedidos[0]
        return 'peguei_pedido'

class VerificaDepositos(smach.State):
    def __init__(self):
	smach.State.__init__(self, outcomes=['verificado', 'verificar'])

    def execute(self, userdata):
	global areas_depositos
	global pub
	global seq
	global pos

	if pos != -1:
		ligarNavigation(areas_depositos[pos], seq, "Deposito")
		cor = buscarPedido(pub)
		areas_depositos[pos][3] = cor
		store_areas_depositos[pos][3] = cor
		rospy.logwarn("deposito "+str(pos)+" = "+str(areas_depositos[pos][3]))
		pos -= 1
		return 'verificar'

        return 'verificado'

class BuscarProduto(smach.State):
    def __init__(self):
	smach.State.__init__(self, outcomes=['area_produto'])

    def execute(self, userdata):
	buscarProduto()
        return 'area_produto'

class VerificarProduto(smach.State):
    def __init__(self):
	smach.State.__init__(self, outcomes=['correto', 'errado'],				
				input_keys=['produto'],
                                output_keys=['produto'])

    def execute(self, userdata):
	global pedidos
	global area_numero

	resp = verificarProduto(pedidos)
 
	if resp.contain == True:
	    rospy.logwarn("produto correto")
	    userdata.produto = resp.object
	    areas_produtos.pop(area_numero)
            return 'correto'

	area_numero += 1
        rospy.logwarn("produto errado")
	return 'errado'

class LigarLed(smach.State):
    def __init__(self):
	smach.State.__init__(self, outcomes=['led_ligado'])

    def execute(self, userdata):
	global deposito_flag
	global pedidos
	ligarLed(pedidos)
        return 'led_ligado'

class PegarProduto(smach.State):
    def __init__(self):
	smach.State.__init__(self, outcomes=['peguei'],				
				input_keys=['produto'])

    def execute(self, userdata):
	global produtos

	pegarProduto(userdata.produto)
        return 'peguei'

class EntregarProduto(smach.State):
    def __init__(self):
	smach.State.__init__(self, outcomes=['entregue'])

    def execute(self, userdata):
	entregarProduto()
        return 'entregue'

class SetaDeposito(smach.State):
    def __init__(self):
	smach.State.__init__(self, outcomes=['setado'])

    def execute(self, userdata):
	global deposito_flag
	piscarLed()
	deposito_flag = True
        return 'setado'

class IrParaCasa(smach.State):
    def __init__(self):
	smach.State.__init__(self, outcomes=['cheguei'])

    def execute(self, userdata):
	irParaCasa()
	seta_parametros()
        return 'cheguei'

# main
def seta_order(req):
	global new_order
	new_order = True


def finaliza_prova(req):
    global termina
    termina = True


def main():
    global pub
    global new_order
    global termina
    global area_numero
    global pos
    global areas_depositos

    termina = False
    new_order = False
    area_numero = 0
    pos = len(areas_depositos)-1
    #nh = rospy.init_node('cbr2015_modulo_a_node')
    pub = rospy.Publisher('action', String, queue_size=100)
    rospy.logwarn("comecou")
    #rospy.Subscriber('outcome', String, callback)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['completo'])
    sm.userdata.produto = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('casa', Casa(), 
				transitions={'iniciar':'ligar_navigation', 'termina':'completo'},
				remapping={'produto':'produto'})

        smach.StateMachine.add('ligar_navigation', LigarNavigation(), 
                               transitions={'coleta_pedido':'indo_pedido', 'coleta_produto':'indo_produto', 
			       'voltar_casa':'indo_casa', 'indo_deposito':'cheguei_deposito'})
	
        smach.StateMachine.add('indo_pedido', BuscarPedido(),
			       transitions={'peguei_pedido':'ligar_led', 'voltar_casa':'indo_casa'},
			       remapping={'produto':'produto'})

        smach.StateMachine.add('verificar_depositos', VerificaDepositos(),
			       transitions={'verificado':'ligar_navigation', 'verificar':'verificar_depositos'})

        smach.StateMachine.add('indo_produto', BuscarProduto(),
			       transitions={'area_produto':'ligar_vision'})

        smach.StateMachine.add('ligar_vision', VerificarProduto(),
			       transitions={'correto':'pegar_produto', 'errado':'ligar_navigation'},
			       remapping={'produto':'produto'})

        smach.StateMachine.add('pegar_produto', PegarProduto(),
			       transitions={'peguei':'seta_deposito'},
			       remapping={'produto':'produto'})

        smach.StateMachine.add('ligar_led', LigarLed(),
			       transitions={'led_ligado':'verificar_depositos'})

        smach.StateMachine.add('cheguei_deposito', EntregarProduto(),
			       transitions={'entregue':'ligar_navigation'})

        smach.StateMachine.add('seta_deposito', SetaDeposito(),
			       transitions={'setado':'ligar_navigation'})

        smach.StateMachine.add('indo_casa', IrParaCasa(),
			       transitions={'cheguei':'casa'})


    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

	# Execute the state machine
    outcome = sm.execute()

	# Wait for ctrl-c to stop the application
    sis.stop()

def seta_parametros():

    global areas_produtos
    areas_produtos = []

    global areas_depositos
    areas_depositos = []

    global store_areas_depositos
    store_areas_depositos = []

    Areas.AREA1[0] = rospy.get_param("/cbr2015_modulo_a_node/area1_x")
    Areas.AREA1[1] = rospy.get_param("/cbr2015_modulo_a_node/area1_y")
    Areas.AREA1[2] = rospy.get_param("/cbr2015_modulo_a_node/area1_orientation")
    Areas.AREA1[3] = None

    Areas.AREA2[0] = rospy.get_param("/cbr2015_modulo_a_node/area2_x")
    Areas.AREA2[1] = rospy.get_param("/cbr2015_modulo_a_node/area2_y")
    Areas.AREA2[2] = rospy.get_param("/cbr2015_modulo_a_node/area2_orientation")
    Areas.AREA2[3] = None

    Areas.AREA3[0] = rospy.get_param("/cbr2015_modulo_a_node/area3_x")
    Areas.AREA3[1] = rospy.get_param("/cbr2015_modulo_a_node/area3_y")
    Areas.AREA3[2] = rospy.get_param("/cbr2015_modulo_a_node/area3_orientation")
    Areas.AREA3[3] = None

    Areas.AREA4[0] = rospy.get_param("/cbr2015_modulo_a_node/area4_x")
    Areas.AREA4[1] = rospy.get_param("/cbr2015_modulo_a_node/area4_y")
    Areas.AREA4[2] = rospy.get_param("/cbr2015_modulo_a_node/area4_orientation")
    Areas.AREA4[3] = None

    Areas.AREA5[0] = rospy.get_param("/cbr2015_modulo_a_node/area5_x")
    Areas.AREA5[1] = rospy.get_param("/cbr2015_modulo_a_node/area5_y")
    Areas.AREA5[2] = rospy.get_param("/cbr2015_modulo_a_node/area5_orientation")
    Areas.AREA5[3] = None

    Areas.AREA6[0] = rospy.get_param("/cbr2015_modulo_a_node/area6_x")
    Areas.AREA6[1] = rospy.get_param("/cbr2015_modulo_a_node/area6_y")
    Areas.AREA6[2] = rospy.get_param("/cbr2015_modulo_a_node/area6_orientation")
    Areas.AREA6[3] = None

    areas_produtos.append(Areas.AREA1)
    areas_produtos.append(Areas.AREA2)
    areas_produtos.append(Areas.AREA3)
    areas_produtos.append(Areas.AREA4)
    areas_produtos.append(Areas.AREA5)
    areas_produtos.append(Areas.AREA6)

    Areas.CASA[0] = rospy.get_param("/cbr2015_modulo_a_node/casa_x")
    Areas.CASA[1] = rospy.get_param("/cbr2015_modulo_a_node/casa_y")
    Areas.CASA[2] = rospy.get_param("/cbr2015_modulo_a_node/casa_orientation")

    Areas.DEPOSITO1[0] = rospy.get_param("/cbr2015_modulo_a_node/deposito1_x")
    Areas.DEPOSITO1[1] = rospy.get_param("/cbr2015_modulo_a_node/deposito1_y")
    Areas.DEPOSITO1[2] = rospy.get_param("/cbr2015_modulo_a_node/deposito1_orientation")
    Areas.DEPOSITO1[3] = None

    Areas.DEPOSITO2[0] = rospy.get_param("/cbr2015_modulo_a_node/deposito2_x")
    Areas.DEPOSITO2[1] = rospy.get_param("/cbr2015_modulo_a_node/deposito2_y")
    Areas.DEPOSITO2[2] = rospy.get_param("/cbr2015_modulo_a_node/deposito2_orientation")
    Areas.DEPOSITO2[3] = None

    Areas.DEPOSITO3[0] = rospy.get_param("/cbr2015_modulo_a_node/deposito3_x")
    Areas.DEPOSITO3[1] = rospy.get_param("/cbr2015_modulo_a_node/deposito3_y")
    Areas.DEPOSITO3[2] = rospy.get_param("/cbr2015_modulo_a_node/deposito3_orientation")
    Areas.DEPOSITO3[3] = None

    areas_depositos.append(Areas.DEPOSITO1)
    areas_depositos.append(Areas.DEPOSITO2)
    areas_depositos.append(Areas.DEPOSITO3)

    Areas.STORE_DEPOSITO1[0] = rospy.get_param("/cbr2015_modulo_a_node/store_deposito1_x")
    Areas.STORE_DEPOSITO1[1] = rospy.get_param("/cbr2015_modulo_a_node/store_deposito1_y")
    Areas.STORE_DEPOSITO1[2] = rospy.get_param("/cbr2015_modulo_a_node/store_deposito1_orientation")
    Areas.STORE_DEPOSITO1[3] = None

    Areas.STORE_DEPOSITO2[0] = rospy.get_param("/cbr2015_modulo_a_node/store_deposito2_x")
    Areas.STORE_DEPOSITO2[1] = rospy.get_param("/cbr2015_modulo_a_node/store_deposito2_y")
    Areas.STORE_DEPOSITO2[2] = rospy.get_param("/cbr2015_modulo_a_node/store_deposito2_orientation")
    Areas.STORE_DEPOSITO2[3] = None

    Areas.STORE_DEPOSITO3[0] = rospy.get_param("/cbr2015_modulo_a_node/store_deposito3_x")
    Areas.STORE_DEPOSITO3[1] = rospy.get_param("/cbr2015_modulo_a_node/store_deposito3_y")
    Areas.STORE_DEPOSITO3[2] = rospy.get_param("/cbr2015_modulo_a_node/store_deposito3_orientation")
    Areas.STORE_DEPOSITO3[3] = None

    store_areas_depositos.append(Areas.STORE_DEPOSITO1)
    store_areas_depositos.append(Areas.STORE_DEPOSITO2)
    store_areas_depositos.append(Areas.STORE_DEPOSITO3)

    Areas.PEDIDOS[0] = rospy.get_param("/cbr2015_modulo_a_node/pedido_x")
    Areas.PEDIDOS[1] = rospy.get_param("/cbr2015_modulo_a_node/pedido_y")
    Areas.PEDIDOS[2] = rospy.get_param("/cbr2015_modulo_a_node/pedido_orientation")

def voltar(req):
	ligarNavigation(Areas.CASA, seq, "Casa")
	
	sys.exit()
	

if __name__ == '__main__':
    rospy.init_node('cbr2015_modulo_a_node')
    rospy.loginfo("cbr2015_modula_a node is up and running!!!")
    s = rospy.Service('voltar_para_casa', Empty, voltar)
    s = rospy.Service('new_order', Empty, seta_order)
    s = rospy.Service('finaliza_prova', Empty, finaliza_prova)
    rospy.Subscriber("cmd_vel", Twist, atualizaCmdVel)
    #s = rospy.Service('get_started', Empty, main)    
    seta_parametros()
    main()

    rospy.spin()

