import random
import time
import sys

class Grafo:

	# Construtor padrao
	def __init__(self):
		self.G = {}
		# Usado na funcao procuraFechoTransitivo()
		self.jaVisitados = {}

	# Construtor que usa o arquivo grafo.dat para gerar o grafo
	def __init__(self, conexoes):
		self.G = {}
		# Usado na funcao procuraFechoTransitivo()
		self.jaVisitados = {}
		self.hospital = []
		self.dist  = []
		self.nextV = []
		# Percorre o vetor "conexoes", criando o grafo a partir dele
		for x in range(0, len(conexoes)):
			for y in range(2):
				self.adicionaVertice(conexoes[x][y])
			self.conecta(conexoes[x][0], conexoes[x][1], conexoes[x][2])

	# Construtor que usa o arquivo das ambulancias para gerar o grafo
	def __init__(self, conexoes, length):
		self.G = {}
		# Usado na funcao procuraFechoTransitivo()
		self.jaVisitados = {}
		# Percorre o vetor "conexoes", criando o grafo a partir dele
		self.hospital = []
		self.hospital.append(conexoes[length][0])
		self.hospital.append(conexoes[length][1])
		for x in range(0, length):
			for y in range(2):
				self.adicionaVertice(conexoes[x][y])
			self.conecta(conexoes[x][0], conexoes[x][1], conexoes[x][2])

	# Adiciona vertice, caso nao exista algum com mesmo nome
	def adicionaVertice(self, v):
		if not self.G.has_key(v):
			self.G[v] = []

	# Remove um vertice, primeiro remove as "arestas" que contem esse nodo e
	# depois apaga o nodo em si
	def removeVertice(self, v):
		#Remover arestas que chegam no vertice removido
		for x in self.G.keys():
			for y in range(0, len(self.G[x])):
				if self.G[x][y][0] == v:
					self.G[x].remove(self.G[x][y])
		#Remover vertice
		if self.G.has_key(v):
			del self.G[v]

	# Cria uma aresta que liga nodo1 com nodo2
	def conecta(self, v1, v2, peso):
		if self.G.has_key(v1) and self.G.has_key(v2):
			self.G[v1].append([v2, peso])
			self.G[v2].append([v1, peso])

	# Remove a aresta que liga o nodo1 com o nodo2
	def desconecta(self, v1, v2, peso):
		if self.G.has_key(v1) and self.G.has_key(v2):
			self.G[v1].remove([v2, peso])
			self.G[v2].remove([v1, peso])

	# Retorna o numero de vertices do Grafo
	def ordem(self):
		return len(self.G)

	# Retorna um conjunto contendo os vertices do Grafo
	def vertices(self):
		return self.G.keys()

	# Retorna um vertice aleatorio do Grafo
	def umVertice(self):
		index = random.randint(0, len(self.G)-1)
		return self.G.keys()[index]

	# Retorna um conjunto contendo os vertices adjacentes ao nodo no Grafo
	def adjacentes(self, v):
		adjacentes = []
		for x in range(0, len(self.G[v])):
				adjacentes.append(self.G[v][x][0])
		return adjacentes

	# Retorna o numero de vertices adjacentes ao nodo no Grafo
	def grau(self, v):
		return len(self.G[v])



	# Verifica se todos os vertices do grafo possuem o mesmo grau
	def eRegular(self):
		_grau = self.grau(self.G.keys()[0])
		for x in range(1, len(self.G)):
			if self.grau(self.G.keys()[x]) != (_grau):
				return False
		return True

	# Verifica se cada vertice do grafo esta conectado a todos
	# os outros vertices
	def eCompleto(self):
		n = self.ordem()-1
		for x in self.G.keys():
				if self.grau(x) != n:
					return False
		return True

	# Retorna um conjunto contendo todos os vertices de G que
	# sao transitivamente alcancaveis partindo de v
	def fechoTransitivo(self, v):
		if v not in self.G:
			return None
		fT = self.procuraFechoTransitivo(v, {})
		self.jaVisitados = {}
		return fT

	# Utilizado por fechoTransitivo
	def procuraFechoTransitivo(self, v, conjunto):
		ft = conjunto
		self.jaVisitados[v] = []
		for vAdj in self.adjacentes(v):
			if vAdj not in self.jaVisitados:
				ft.update(self.procuraFechoTransitivo(vAdj, self.jaVisitados))
		return ft

	# Verifica se existe pelo menos um caminho entre
	# cada par de vertices do grafo
	def eConexo(self):
		vertices = self.G.keys()
		alcancaveis = self.fechoTransitivo(self.umVertice())
		return self.equals(vertices, alcancaveis)

	# Usado por eConexo(), verifica se duas listas possuem os mesmos elementos (iguais, porem nao
	# necessariamente na mesma ordem)
	def equals(self, conj1, conj2):
		for x in conj1:
			for y in conj2:
				if x not in conj2 or y not in conj1:
					return False
		return True

	# Verifica se o grafo e uma arvore, ou seja, se nao possue ciclos se e conexo
	def eArvore(self):
		v = self.umVertice()
		return self.eConexo and not self.haCicloCom(v, v, [])

	# Usado por eArvore(), verifica se v faz parte de algu ciclo no grafo
	def haCicloCom(self, v, vAnterior, jaVisitados):
		if v in jaVisitados:
			return True
		jaVisitados.append(v)
		for vAdj in self.adjacentes(v):
			if vAdj != vAnterior:
				if self.haCicloCom(vAdj, v, jaVisitados):
					return True
		jaVisitados.remove(v)
		return False


	def extendedFloyd(self):
		vert = self.vertices()
		ordem = self.ordem()
		self.dist  = [[self.distance(vert[y], vert[x]) for x in range(ordem)] for y in range(ordem)]
		self.nextV = [[self.vertex(vert[y], vert[x]) for x in range(ordem)] for y in range(ordem)]
		for k in range(0, ordem):
			for i in range(0, ordem):
				for j in range(0, ordem):
					if (self.dist[i][k] + self.dist[k][j]) < self.dist[i][j]:
						self.dist[i][j] = self.dist[i][k] + self.dist[k][j]
						self.nextV[i][j] = self.nextV[i][k]
		return

	def randPath(self):
		for x in self.vertices():
			for y in range(0, len(self.G[x])):
				self.G[x][y][1] = self.G[x][y][1] * random.uniform(1, 1.5)
		return

	def distance(self, v1, v2):
		if v1 == v2:
			return 0
		for x in range(0, len(self.G[v1])):
			if self.G[v1][x][0] == v2:
				return self.G[v1][x][1]
		return float("inf")

	def vertex(self, v1, v2):
		if v1 == v2:
			return v1
		for x in range(0, len(self.G[v1])):
			if self.G[v1][x][0] == v2:
				return self.G[v1][x][0]
		return 'NULL'

	def getIndex(self, v1):
		vert = self.vertices()
		for x in range(0, len(vert)):
			if vert[x] == v1:
				return x
		


	# Encontra o menor caminho entre o nodo1 e nodo2
	def shortestPath(self, inicio, fim, caminho=[]):
		caminho = caminho + [inicio]
		if inicio == fim:
			return caminho
		if inicio not in self.G:
			return None
		menor = None
		for nodo in self.adjacentes(inicio):
			if nodo not in caminho:
				novoCaminho = self.shortestPath(nodo, fim, caminho)
				if novoCaminho:
					if not menor or len(novoCaminho) < len(menor):
						menor = novoCaminho
		return menor

	# Depth First Search, usa recursividade
	def depthFirstSearch(self, inicio, caminho = []):
		caminho = caminho + [inicio]
		for nodo in self.adjacentes(inicio):
			if not nodo in caminho:
				caminho = self.depthFirstSearch(nodo, caminho)
		return caminho

	# Breadth First Search, iterativo
	def breadthFirstSearch(self, inicio, caminho = []):
		stack = [inicio]
		while stack:
			v = stack.pop(0)
			if not v in caminho:
				caminho = caminho + [v]
				stack = stack + self.adjacentes(v)
		return caminho

	def caminhoMinimoHospital(self, v):
		distance = float('inf')
		for x in range(len(self.hospital)):
			if distance > self.dist[self.getIndex(self.hospital[x])][self.getIndex(v)]:
				distance = self.dist[self.getIndex(self.hospital[x])][self.getIndex(v)]
				hosp = self.getIndex(self.hospital[x])
		path = self.getCaminho(hosp, v)
		return distance, path
			
	def getCaminho(self, vA, vE):
		path = [vA]
		while (path[len(path)-1] != vE):
			path.append(self.nextV[self.getIndex(path[len(path)-1])][self.getIndex(vE)])
		return path

def carregarGrafo(arq):
	exec open(arq).read()
	length = len(arestas)
	return Grafo(arestas, length-1)

g = carregarGrafo('ambulan2.dat')
while True:
	print "insira o vertice da emergencia:"
	print g.vertices()
	inpt = input()
	print ""
	print ""
	t1 = time.time()
	g.randPath()
	g.extendedFloyd()
	distanciaX, caminhoX = g.caminhoMinimoHospital(inpt)
	t2 = time.time()
	print "Tempo:"
	print(t2-t1)
	print "Distancia:"
	print distanciaX
	print "Caminho:"
	print caminhoX
	print ""
	print ""

#g = Grafo()
#g.adicionaVertice("A")
#g.adicionaVertice("B")
#g.conecta("A", "B", 10)
#g.adicionaVertice("C")
#g.conecta("B", "C", 2)
#g.conecta("C", "A", 5)
#print g.G


#print "Adjacentes ao vertice A: "
#print g.adjacentes("A")
#g.desconecta("A", "B", 10)
#print g.G
#print g.vertices()
#print g.umVertice()
#print g.grau("B")



#print g.eRegular()
#g.desconecta("A", "C", 5)
#print g.G
#print g.eRegular()

