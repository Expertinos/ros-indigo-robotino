class Cores():
	AMARELO = 1
	AZUL = 2
	VERDE = 3
	VERMELHO = 4

class Objetos():
	AZUL_UM = ['AZUL_UM', Cores.AZUL, 1] #primeiro campo : COR : 2-Azul; 4-Vermelho #segundo campo : numero seringas
	AZUL_TRES = ['AZUL_TRES', Cores.AZUL, 3]	
	AZUL_CINCO = ['AZUL_CINCO',Cores.AZUL, 5]	
	VERMELHO_UM = ['VERMELHO_UM', Cores.VERMELHO, 1]
	VERMELHO_TRES = ['VERMELHO_TRES', Cores.VERMELHO, 3]	
	VERMELHO_CINCO = ['VERMELHO_CINCO', Cores.VERMELHO, 5]
	NONE = ['NONE', 0, 0]


class Areas():
	CASA = ['CASA', 0, 0, 0, '']
	BUFFER = ['BUFFER', 0, 0, 0, '']
	A1 = ['A1', 0, 0, 0, '']
	A2 = ['A2', 0, 0, 0, '']
	A3 = ['A3', 0, 0, 0, '']
	A4 = ['A4', 0, 0, 0, '']
	B1 = ['B1', 0, 0, 0, '']
	B2 = ['B2', 0, 0, 0, '']
	B3 = ['B3', 0, 0, 0, '']
	B4 = ['B4', 0, 0, 0, '']

class AreasCores():
	A1 = ['A1', Cores.VERDE, Cores.VERDE]
	A2 = ['A2', Cores.AMARELO, Cores.AMARELO]
	A3 = ['A3', Cores.VERMELHO, Cores.VERMELHO]
	A4 = ['A4', Cores.VERDE, Cores.AMARELO]
	B1 = ['B1', Cores.VERDE, Cores.VERMELHO]
	B2 = ['B2', Cores.AMARELO, Cores.VERMELHO]
	B3 = ['B3', Cores.AMARELO, Cores.VERDE]
	B4 = ['B4', Cores.VERMELHO, Cores.VERDE]

class AreasOrganizadas():
	A1 = ['A1', Objetos.AZUL_TRES]
	A2 = ['A2', Objetos.VERMELHO_CINCO]
	A3 = ['A3', Objetos.VERMELHO_UM]
	A4 = ['A4', Objetos.AZUL_UM]
	B1 = ['B1', Objetos.AZUL_CINCO]
	B2 = ['B2', Objetos.VERMELHO_TRES]
	B3 = ['B3', Objetos.VERMELHO_TRES]
	B4 = ['B4', Objetos.AZUL_TRES]

def areaOrganizada(area, objeto):
	if area[0] == Areas.A1[0]:
		return area[4] == AreasOrganizadas.A1[1] or objeto == AreasOrganizadas.A1[1]
	elif area[0] == Areas.A2[0]:
		return area[4] == AreasOrganizadas.A2[1] or objeto == AreasOrganizadas.A2[1]
	elif area[0] == Areas.A3[0]:
		return area[4] == AreasOrganizadas.A3[1] or objeto == AreasOrganizadas.A3[1]
	elif area[0] == Areas.A4[0]:
		return area[4] == AreasOrganizadas.A4[1] or objeto == AreasOrganizadas.A4[1]
	elif area[0] == Areas.B1[0]:
		return area[4] == AreasOrganizadas.B1[1] or objeto == AreasOrganizadas.B1[1]
	elif area[0] == Areas.B2[0]:
		return area[4] == AreasOrganizadas.B2[1] or objeto == AreasOrganizadas.B2[1]
	elif area[0] == Areas.B3[0]:
		return area[4] == AreasOrganizadas.B3[1] or objeto == AreasOrganizadas.B3[1]
	elif area[0] == Areas.B4[0]:
		return area[4] == AreasOrganizadas.B4[1] or objeto == AreasOrganizadas.B4[1]

def areaComObjDesejado(areaDes, areaAux):
	if areaDes[0] == Areas.A1[0]:
		return areaAux[4] == AreasOrganizadas.A1[1]
	elif areaDes[0] == Areas.A2[0]:
		return areaAux[4] == AreasOrganizadas.A2[1]
	elif areaDes[0] == Areas.A3[0]:
		return areaAux[4] == AreasOrganizadas.A3[1]
	elif areaDes[0] == Areas.A4[0]:
		return areaAux[4] == AreasOrganizadas.A4[1]
	elif areaDes[0] == Areas.B1[0]:
		return areaAux[4] == AreasOrganizadas.B1[1]
	elif areaDes[0] == Areas.B2[0]:
		return areaAux[4] == AreasOrganizadas.B2[1]
	elif areaDes[0] == Areas.B3[0]:
		return areaAux[4] == AreasOrganizadas.B3[1]
	elif areaDes[0] == Areas.B4[0]:
		return areaAux[4] == AreasOrganizadas.B4[1]

def areaDesorganizada(area):
	if area[0] == Areas.A1[0]:
		return area[4] != AreasOrganizadas.A1[1]
	elif area[0] == Areas.A2[0]:
		return area[4] != AreasOrganizadas.A2[1]
	elif area[0] == Areas.A3[0]:
		return area[4] != AreasOrganizadas.A3[1]
	elif area[0] == Areas.A4[0]:
		return area[4] != AreasOrganizadas.A4[1]
	elif area[0] == Areas.B1[0]:
		return area[4] != AreasOrganizadas.B1[1]
	elif area[0] == Areas.B2[0]:
		return area[4] != AreasOrganizadas.B2[1]
	elif area[0] == Areas.B3[0]:
		return area[4] != AreasOrganizadas.B3[1]
	elif area[0] == Areas.B4[0]:
		return area[4] != AreasOrganizadas.B4[1]

def atualizaArea(area, objeto):
	if area[0] == Areas.A1[0]:
		Areas.A1[4] = objeto
		return
	elif area[0] == Areas.A2[0]:
		Areas.A2[4] = objeto
		return	
	elif area[0] == Areas.A3[0]:
		Areas.A3[4] = objeto
		return
	elif area[0] == Areas.A4[0]:
		Areas.A4[4] = objeto
		return
	elif area[0] == Areas.B1[0]:
		Areas.B1[4] = objeto
		return
	elif area[0] == Areas.B2[0]:
		Areas.B2[4] = objeto
		return
	elif area[0] == Areas.B3[0]:
		Areas.B3[4] = objeto
		return
	elif area[0] == Areas.B4[0]:
		Areas.B4[4] = objeto
		return
	elif area[0] == Areas.BUFFER[0]:
		Areas.BUFFER[4] = objeto
		return

#Vai ser usado so enquanto nao tem a leitura dos postes
def sinalizaLeitura(prox_area):
	cores = [0, 0]
	if prox_area[0] == AreasCores.A1[0]:
		cores[0] = AreasCores.A1[1]
		cores[1] = AreasCores.A1[2]
		return cores
	elif prox_area[0] == AreasCores.A2[0]:
		cores[0] = AreasCores.A2[1]
		cores[1] = AreasCores.A2[2]
		return cores
	elif prox_area[0] == AreasCores.A3[0]:
		cores[0] = AreasCores.A3[1]
		cores[1] = AreasCores.A3[2]
		return cores
	elif prox_area[0] == AreasCores.A4[0]:
		cores[0] = AreasCores.A4[1]
		cores[1] = AreasCores.A4[2]
		return cores
	elif prox_area[0] == AreasCores.B1[0]:
		cores[0] = AreasCores.B1[1]
		cores[1] = AreasCores.B1[2]
		return cores
	elif prox_area[0] == AreasCores.B2[0]:
		cores[0] = AreasCores.B2[1]
		cores[1] = AreasCores.B2[2]
		return cores
	elif prox_area[0] == AreasCores.B3[0]:
		cores[0] = AreasCores.B3[1]
		cores[1] = AreasCores.B3[2]
		return cores
	elif prox_area[0] == AreasCores.B4[0]:
		cores[0] = AreasCores.B4[1]
		cores[1] = AreasCores.B4[2]
		return cores
