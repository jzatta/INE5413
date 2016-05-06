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

	# Adiciona vertice, caso nao exista algum com mesmo nome
	def adicionaVertice(self, v):
		if not self.G.has_key(v):
			self.G[v] = []

	# Remove um vertice, primeiro remove as "arestas" que contem esse nodo e
	# depois apaga o nodo em si
	def removeVertice(self, v):

		#Remove arestas que chegam no vertice removido
		for x in self.G.keys():
			for y in range(0, len(self.G[x])):
				if self.G[x][y][0] == v:
					self.G[x].remove(self.G[x][y])
		#Remove vertice
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






	# Usado por dijkstra(). Retorna o vetor fornecido, ordenado.
	def heapsort(self, origem, vertices, n):
		#for i in range(n//2, 1, -1):
		i = (n//2)
		while i >= 0 :
			self.ajuste(origem, vertices, i, n)
			#print vertices
			i = i-1
		#print vertices
		i = (int)(n-1)
		#for i in range(n-1, 1,  -1):
		while i >= 0 :
			temp = vertices[i+1]
			vertices[i+1] = vertices[0]
			vertices[0] = temp
			self.ajuste(origem, vertices, 0, i)
			i = i-1
		return vertices

	# Usado por heapsort(). Leva os maiores valores para cima (sift-up)
	def ajuste(self, origem, vertices, i, n):
		aAux = vertices[i]
		j = 2*i
		while j <= n :
			if j < n and self.pesoAresta(origem, vertices[j]) < self.pesoAresta(origem, vertices[j+1]):
				j = j+1
			if self.pesoAresta(origem, aAux) >= self.pesoAresta(origem, vertices[j]) :
				return
			vertices[j//2] = vertices[j]
			vertices[j] = aAux
			j = 2*j
		vertices[j//2] = aAux

	# Usado por dijkstra(). Retorna um vetor ordenado com heapsort
	def extrairMinimo(self, Q, origem):
		self.ajuste(origem, Q, (len(Q) - 1)//2, len(Q) - 1)
		self.swap(Q)
		return Q[len(Q) - 1]

	def swap(self, Q):
		tmp = Q[len(Q) - 1]
		Q[len(Q) - 1] = Q[0]
		Q[0] = tmp




	# Usado por dijkstra(). Retorna o peso da aresta que conecta os vertices fornecidos
	def pesoAresta(self, u, v):
		adj = self.adjacentes(u)
		if u == v :
			return 0
		for i in range(0, len(adj)):
			if self.G[u][i][0] == v :
				return self.G[u][i][1]
		return float('inf')

	# Algoritmo de custo minimo de Dijkstra. Calcula os custos minimos de um vertice fornecido
	# ate todos os outros vertices. Retorna uma lista com o vertice destino e o custo minimo ate ele
	def dijkstra(self, origem):
		
		adjacentes = self.adjacentes(origem) # Vertices cuja distancia e conhecida
		
		d = {} # Distancias da origem ate cada vertice
		caminhos = {} # Caminho entre de cada vertice para a origem

		for i in range(0, self.ordem()):
			d[self.vertices()[i]] = float('inf')
		d[origem] = 0
		caminhos[origem] = origem

		#S = adjacentes # Vertices cuja distancia minima e conhecida
		S = []

		Q = list(set(self.vertices()) - set(S)) # Fila de prioridade composta por V (todos os vertices) - S
		
		self.heapsort(origem, Q, len(Q) - 1) # Ordena os vertices por distancia e retorna o mais proximo

		# Itera enquanto ha elementos não visitados
		while Q != [] :
			u = self.extrairMinimo(Q, origem)
			Q.remove(u)
			S.append(u)
			adjacentesU = self.adjacentes(u)
			# Itera para todos os vertices adjacentes a u
			for v in adjacentesU :
				# Pega peso da aresta entre os nodos u e v
				pesoAresta = self.pesoAresta(u, v)
				# E compara com o menor caminho encontrado ate o momento
				if d[v] > d[u] + pesoAresta:
					# caso seja menor atualiza o valor
					d[v] = d[u] + pesoAresta
					# E atualiza o caminho
					caminhos[v] = u
		# Retorna as distancias e os caminhos
		return d, caminhos

# Gera o grafo a partir do arquivo .dat contendo as arestas
# e os hospitais
def carregarGrafo(arq):
	exec open(arq).read()
	length = len(arestas)
	return Grafo(arestas)


arqGrafos = ['grafo5.dat', 'grafo20.dat', 'grafo50.dat', 'grafo100.dat', 'grafo200.dat', 'grafo500.dat']
#arqGrafos = ['grafo5.dat']
for arquivo in arqGrafos:
	g = carregarGrafo(arquivo) # Para trocar o .dat lido, basta substituir 'grafo5.dat' aqui pelo novo .dat
	randNodo = g.umVertice()
	#randNodo = 3
	print "Nodo aleatorio:"
	print randNodo
	t1 = time.time()
	d, caminho = g.dijkstra(randNodo)
	t2 = time.time()
	print "As distancias entre o nodo aleatorio e os outros nodos e:"
	print d
	print "Os caminhos entre o nodo aleatorio e os outros nodos:"
	print caminho
	print "O tempo para calcular dijkstra:"
	print(t2-t1)
	print ""
	print ""

