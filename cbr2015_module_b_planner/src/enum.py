class AreasSMORG():
	PRATDES = 0 
	PRATAUX = 1
	BUFFER = 2
	CASA = 3

class ObjetosSMORG():
	OBJDES = 0 #[0,0]
	OBJAUX = 1

class Objetos():
	AZUL_UM = ['AZUL_UM', 2, 1] #primeiro campo : COR : 1-Azul; 2-Vermelho #segundo campo : numero seringas
	AZUL_TRES = ['AZUL_TRES', 2, 3]	
	AZUL_CINCO = ['AZUL_CINCO',2, 5]	
	VERMELHO_UM = ['VERMELHO_UM', 4, 1]
	VERMELHO_TRES = ['VERMELHO_TRES', 4, 3]	
	VERMELHO_CINCO = ['VERMELHO_CINCO', 4, 5]
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
		return area[4] == AreasOrganizadas.A1[1] or objeto == AreasOrganizadas.A2[1]
	elif area[0] == Areas.A3[0]:
		return area[4] == AreasOrganizadas.A1[1] or objeto == AreasOrganizadas.A3[1]
	elif area[0] == Areas.A4[0]:
		return area[4] == AreasOrganizadas.A1[1] or objeto == AreasOrganizadas.A4[1]
	elif area[0] == Areas.B1[0]:
		return area[4] == AreasOrganizadas.A1[1] or objeto == AreasOrganizadas.B1[1]
	elif area[0] == Areas.B2[0]:
		return area[4] == AreasOrganizadas.A1[1] or objeto == AreasOrganizadas.B2[1]
	elif area[0] == Areas.B3[0]:
		return area[4] == AreasOrganizadas.A1[1] or objeto == AreasOrganizadas.B3[1]
	elif area[0] == Areas.B4[0]:
		return area[4] == AreasOrganizadas.A1[1] or objeto == AreasOrganizadas.B4[1]

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
