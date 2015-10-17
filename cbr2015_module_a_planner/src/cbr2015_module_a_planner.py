#!/usr/bin/env python

import roslib; roslib.load_manifest('cbr2015_module_a_planner')
import rospy
import smach
import smach_ros
from Casa import casa
from LigarNavigation import ligarNavigation
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

global area_casa
global area_deposito
global area_pedido
global areas_produtos
produtos = [Product.TV, Product.DVD, Product.CELULAR, Product.TABLET, Product.NOTEBOOK]
global pedidos #= [Product.DVD, Product.CELULAR, Product.TABLET]
led = False
seq = 0

# define state Foo
class Casa(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['iniciar'])
        self.counter = 0

    def execute(self, userdata):
        casa()
        return 'iniciar'

# define state Bar
class LigarNavigation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['coleta_pedido', 'coleta_produto', 'voltar_casa', 'indo_deposito'])
        self.count = 0

    def execute(self, userdata):
	global led
	global seq
	global areas_produtos
	global pedidos

	seq += 1

	if led == True:
		ligarNavigation(Areas.DEPOSITO, seq)
		return 'indo_deposito'

        if self.count == 0:
            self.count += 1
    	    ligarNavigation(Areas.PEDIDOS, seq)
	    return 'coleta_pedido'

	if len(areas_produtos) == 0 or len(pedidos) == 0:
		ligarNavigation(Areas.CASA, seq)
		return 'voltar_casa'

        ligarNavigation(areas_produtos.pop(0), seq)
        return 'coleta_produto'

class BuscarPedido(smach.State):
    def __init__(self):
	smach.State.__init__(self, outcomes=['peguei_pedido'])

    def execute(self, userdata):
	global pub
	global pedidos

	pedidos = buscarPedido(pub)
        return 'peguei_pedido'

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

	resp = verificarProduto(pedidos)
 
	if resp.contain == True:
	    rospy.logwarn("produto correto")
	    userdata.produto = resp.product
            return 'correto'

        rospy.logwarn("produto errado")
	return 'errado'

class LigarLed(smach.State):
    def __init__(self):
	smach.State.__init__(self, outcomes=['led_ligado'])

    def execute(self, userdata):
	global led
	ligarLed()
	led = True
        return 'led_ligado'

class PegarProduto(smach.State):
    def __init__(self):
	smach.State.__init__(self, outcomes=['peguei'],				
				input_keys=['produto'])

    def execute(self, userdata):
	pegarProduto(userdata.produto)
        return 'peguei'

class EntregarProduto(smach.State):
    def __init__(self):
	smach.State.__init__(self, outcomes=['entregue'])

    def execute(self, userdata):
	entregarProduto()
        return 'entregue'

class PiscarLed(smach.State):
    def __init__(self):
	smach.State.__init__(self, outcomes=['led_piscado'])

    def execute(self, userdata):
	global led
	piscarLed()
	led = False
        return 'led_piscado'

class IrParaCasa(smach.State):
    def __init__(self):
	smach.State.__init__(self, outcomes=['cheguei'])

    def execute(self, userdata):
	irParaCasa()
        return 'cheguei'

# main
def main():
    global pub
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
                               transitions={'iniciar':'ligar_navigation'})

        smach.StateMachine.add('ligar_navigation', LigarNavigation(), 
                               transitions={'coleta_pedido':'indo_pedido', 'coleta_produto':'indo_produto', 
			       'voltar_casa':'indo_casa', 'indo_deposito':'cheguei_deposito'})
	
        smach.StateMachine.add('indo_pedido', BuscarPedido(),
			       transitions={'peguei_pedido':'ligar_navigation'})

        smach.StateMachine.add('indo_produto', BuscarProduto(),
			       transitions={'area_produto':'ligar_vision'})

        smach.StateMachine.add('ligar_vision', VerificarProduto(),
			       transitions={'correto':'pegar_produto', 'errado':'ligar_navigation'},
			       remapping={'produto':'produto'})

        smach.StateMachine.add('pegar_produto', PegarProduto(),
			       transitions={'peguei':'ligar_led'},
			       remapping={'produto':'produto'})

        smach.StateMachine.add('ligar_led', LigarLed(),
			       transitions={'led_ligado':'ligar_navigation'})

        smach.StateMachine.add('cheguei_deposito', EntregarProduto(),
			       transitions={'entregue':'piscar_led'})

        smach.StateMachine.add('piscar_led', PiscarLed(),
			       transitions={'led_piscado':'ligar_navigation'})

        smach.StateMachine.add('indo_casa', IrParaCasa(),
			       transitions={'cheguei':'completo'})


    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

	# Execute the state machine
    outcome = sm.execute()

	# Wait for ctrl-c to stop the application
    sis.stop()

def seta_parametros():

    global areas_produtos
    areas_produtos = []

    Areas.AREA1[1] = rospy.get_param("/cbr2015_modulo_a_node/area1_x")
    Areas.AREA1[2] = rospy.get_param("/cbr2015_modulo_a_node/area1_y")
    Areas.AREA1[3] = rospy.get_param("/cbr2015_modulo_a_node/area1_orientation")

    Areas.AREA2[1] = rospy.get_param("/cbr2015_modulo_a_node/area2_x")
    Areas.AREA2[2] = rospy.get_param("/cbr2015_modulo_a_node/area2_y")
    Areas.AREA2[3] = rospy.get_param("/cbr2015_modulo_a_node/area2_orientation")

    Areas.AREA3[1] = rospy.get_param("/cbr2015_modulo_a_node/area3_x")
    Areas.AREA3[2] = rospy.get_param("/cbr2015_modulo_a_node/area3_y")
    Areas.AREA3[3] = rospy.get_param("/cbr2015_modulo_a_node/area3_orientation")

    Areas.AREA4[1] = rospy.get_param("/cbr2015_modulo_a_node/area4_x")
    Areas.AREA4[2] = rospy.get_param("/cbr2015_modulo_a_node/area4_y")
    Areas.AREA4[3] = rospy.get_param("/cbr2015_modulo_a_node/area4_orientation")

    Areas.AREA5[1] = rospy.get_param("/cbr2015_modulo_a_node/area5_x")
    Areas.AREA5[2] = rospy.get_param("/cbr2015_modulo_a_node/area5_y")
    Areas.AREA5[3] = rospy.get_param("/cbr2015_modulo_a_node/area5_orientation")

    areas_produtos.append(Areas.AREA1)
    areas_produtos.append(Areas.AREA2)
    areas_produtos.append(Areas.AREA3)
    areas_produtos.append(Areas.AREA4)
    areas_produtos.append(Areas.AREA5)

    Areas.CASA[1] = rospy.get_param("/cbr2015_modulo_a_node/casa_x")
    Areas.CASA[2] = rospy.get_param("/cbr2015_modulo_a_node/casa_y")
    Areas.CASA[3] = rospy.get_param("/cbr2015_modulo_a_node/casa_orientation")

    Areas.DEPOSITO[1] = rospy.get_param("/cbr2015_modulo_a_node/deposito_x")
    Areas.DEPOSITO[2] = rospy.get_param("/cbr2015_modulo_a_node/deposito_y")
    Areas.DEPOSITO[3] = rospy.get_param("/cbr2015_modulo_a_node/deposito_orientation")

    Areas.PEDIDOS[1] = rospy.get_param("/cbr2015_modulo_a_node/pedido_x")
    Areas.PEDIDOS[2] = rospy.get_param("/cbr2015_modulo_a_node/pedido_y")
    Areas.PEDIDOS[3] = rospy.get_param("/cbr2015_modulo_a_node/pedido_orientation")

if __name__ == '__main__':
    rospy.init_node('cbr2015_modulo_a_node')
    rospy.loginfo("cbr2015_modula_a node is up and running!!!")
    #s = rospy.Service('get_started', Empty, main)    
    seta_parametros()
    main()

    rospy.spin()

