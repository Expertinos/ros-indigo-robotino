#!/usr/bin/env python

import roslib; roslib.load_manifest('planejamento_cbr')
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

area_casa = [0, 0]
area_deposito = [300, 300]
area_pedido = [50, 50]
areas = [[10,20], [100, 200], [50, 100], [100, 50], [200, 200]]
produtos = [Product.TV, Product.DVD, Product.CELULAR, Product.TABLET, Product.NOTEBOOK]
pedidos = [Product.DVD, Product.CELULAR, Product.TABLET]
led = False

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

	if led == True:
		ligarNavigation(area_deposito)
		return 'indo_deposito'

	if len(produtos) == 0 or len(pedidos) == 0:
		ligarNavigation(area_casa)
		return 'voltar_casa'

        if self.count == 0:
            self.count += 1
    	    ligarNavigation(area_pedido)
	    return 'coleta_pedido'

        ligarNavigation(areas.pop(0))
        return 'coleta_produto'

class BuscarPedido(smach.State):
    def __init__(self):
	smach.State.__init__(self, outcomes=['peguei_pedido'])

    def execute(self, userdata):
	global pub

	buscarPedido(pub)
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
	verificarProduto()
        produto = produtos.pop(0)
	if produto in pedidos:
	    pedidos.remove(produto)
	    userdata.produto = produto
            return 'correto'

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
    nh = rospy.init_node('planejamento_cbr')
    pub = rospy.Publisher('action', String, queue_size=100)
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
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()

