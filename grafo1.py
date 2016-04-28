import random
import time

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
		# Percorre o vetor "conexoes", criando o grafo a partir dele
		for x in range(0, len(conexoes)):
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

	# Usado por eConexo(), verifica se duas listas possuem os mesmos elementos (iguais, porem nao
	# necessariamente na mesma ordem)
	def equals(self, conj1, conj2):
		for x in conj1:
			for y in conj2:
				if x not in conj2 or y not in conj1:
					return False
		return True

t1 = time.time()

arq = open('grafo.dat')
texto = arq.read()
exec texto

g = Grafo(conexoes)
#g = Grafo()

print g.G

t2 = time.time()

print(t2-t1)

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

