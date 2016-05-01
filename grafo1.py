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


	# Calcula todos os caminhos e distancias entre cada vertice, usando e atualizando
	# as matrizes de roteamento e distancias. (Algoritmo de Floyd extendido)
	def extendedFloyd(self):
		vert = self.vertices()
		ordem = self.ordem()

		# Gera a matriz de distancias (inicial) a partir das informacoes do grafo
		self.dist  = [[self.distance(vert[y], vert[x]) for x in range(ordem)] for y in range(ordem)]

		# Gera a matriz de roteamento (inicial) a partir das informacoes do grafo
		self.nextV = [[self.vertex(vert[y], vert[x]) for x in range(ordem)] for y in range(ordem)]

		# Atualiza as matrizes de distancia e roteamento ate elas possuirem as distancias/rotas
		# minimas de cada vertice para cada outro vertice
		for k in range(0, ordem):
			for i in range(0, ordem):
				for j in range(0, ordem):
					if (self.dist[i][k] + self.dist[k][j]) < self.dist[i][j]:
						self.dist[i][j] = self.dist[i][k] + self.dist[k][j]
						self.nextV[i][j] = self.nextV[i][k]
		return

	# Gera pesos aleatorios para todas as arestas do grafo
	def randPath(self):
		for x in self.vertices():
			for y in range(0, len(self.G[x])):
				self.G[x][y][1] = self.G[x][y][1] * random.uniform(0.5, 1.5)
		return

	# Usado por extendedFloyd(). Busca a distancia entre dois vertices do grafo,
	# para gerar matriz de distancia do Floyd extendido
	def distance(self, v1, v2):
		if v1 == v2:
			return 0
		for x in range(0, len(self.G[v1])):
			if self.G[v1][x][0] == v2:
				return self.G[v1][x][1]
		return float("inf")

	# Usada por extendedFloyd(). Retorna cada vertice adjacente a cada vertice
	# do grafo. Usada para gerar a matriz de roteamento
	def vertex(self, v1, v2):
		if v1 == v2:
			return v1
		for x in range(0, len(self.G[v1])):
			if self.G[v1][x][0] == v2:
				return self.G[v1][x][0]
		return 'NULL'

	# Retorna o indice relativo ao vertice buscado no grafo
	# (Usado pelo algoritmo de Floyd extendido)
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

	# Usa a matriz de distancias para determinar o hospital mais proximo da emergencia,
	# e usa a matiz de rotas para determinar a rota desse caminho
	def caminhoMinimoHospital(self, v):
		distance = float('inf')
		for x in range(len(self.hospital)):
			if distance > self.dist[self.getIndex(self.hospital[x])][self.getIndex(v)]:
				distance = self.dist[self.getIndex(self.hospital[x])][self.getIndex(v)]
				hosp = self.getIndex(self.hospital[x])
		path = self.getCaminho(hosp, v)
		return distance, path

	# Usada em caminhoMinimoHospital(). Usa a matriz de roteamento para obter o caminho do hospital mais proximo
	# (ja obtido) ate a emergencia
	def getCaminho(self, vA, vE):
		path = [vA]
		while (path[len(path)-1] != vE):
			path.append(self.nextV[self.getIndex(path[len(path)-1])][self.getIndex(vE)])
		return path

	# Usado por dijkstra(). Retorna o vetor fornecido, ordenado.
	def heapsort(self, vertices, n):
		#for i in range(n//2, 1, -1):
		i = (n//2)
		while i >= 0 :
			self.ajuste(vertices, i, n)
			#print vertices
			i = i-1
		#print vertices
		i = (int)(n-1)
		#for i in range(n-1, 1,  -1):
		while i >= 0 :
			temp = vertices[i+1]
			vertices[i+1] = vertices[0]
			vertices[0] = temp
			self.ajuste(vertices, 0, i)
			i = i-1
		return vertices

	# Usado por heapsort(). Leva os maiores valores para cima (sift-up)
	def ajuste(self, vertices, i, n):
		aAux = vertices[i]
		j = 2*i
		while j <= n :
			if j < n and vertices[j] < vertices[j+1]:
				j = j+1
			if aAux >= vertices[j] :
				return
			vertices[j//2] = vertices[j]
			vertices[j] = aAux
			j = 2*j
		vertices[j//2] = aAux

	# Usado por dijkstra(). Retorna um vetor ordenado com heapsort
	def extrairMinimo(self, Q):
		return self.heapsort(Q, len(Q) - 1)[0] # Ordena os vertices por distancia e retorna o mais proximo
		

	# Usado por dijkstra(). Retorna o peso da aresta que conecta os vertices fornecidos
	def pesoAresta(self, u, v):
		adj = self.adjacentes(u)
		if u == v :
			return 0
		for i in range(0, len(adj)):
			if self.G[u][i][0] == v :
				return self.G[u][i][1]
		return 99999999

	# Algoritmo de custo minimo de Dijkstra. Calcula os custos minimos de um vertice fornecido
	# ate todos os outros vertices. Retorna uma lista com o vertice destino e o custo minimo ate ele
	def dijkstra(self, origem):
		
		adjacentes = self.adjacentes(origem) # Vertices cuja distancia e conhecida
		print adjacentes #1
		
		d = {} # Distancias da origem ate cada vertice

		d[origem] = 0

		for i in range(0, self.ordem()):
			d[self.vertices()[i]] = self.pesoAresta(origem, self.vertices()[i])

		print d
		S = adjacentes # Vertices cuja distancia minima e conhecida
		print "Vertices com distancia conhecida: "
		print S #2

		Q = list(set(self.vertices()) - set(S)) # Fila de prioridade composta por V (todos os vertices) - S
		print "fila de prioridades dos vertices ainda sem peso nas arestas: "
		print Q #3

		while Q != [] :
			u = self.extrairMinimo(Q)
			print "Minimo extraido (dos vertices ainda sem distancia calculada):"
			print u #4
			Q.remove(u)
			S.append(u)
			adjacentesU = self.adjacentes(u)
			print "Adjacentes ao extraido: "
			print adjacentesU #5
			for i in range(0, len(adjacentesU)) :
				v = adjacentesU[i]
				print 'adjacente escolhido'
				print d[v]
				pesoAresta = self.pesoAresta(u, v)
				print d[u] + pesoAresta
				if d[u] > d[v] + pesoAresta:
					d[u] = d[v] + pesoAresta
		return d

# Gera o grafo a partir do arquivo .dat contendo as arestas
# e os hospitais
def carregarGrafo(arq):
	exec open(arq).read()
	length = len(arestas)
	return Grafo(arestas)


g = carregarGrafo('grafo5.dat') # Para trocar o .dat lido, basta substituir 'grafo5.dat' aqui pelo novo .dat

#print g.G

#print g.vertices()


d = g.dijkstra(2)
print "As distancias entre o nodo 2 e os outros nodos e:"
print d

#random.shuffle(adjacentes)

#adjacentes.sort()


#while True:
	# Aleatoriza os pesos das arestas, com base nos pesos originais
	#g.randPath()
	
	#print "Insira o vertice da emergencia:"
	#print g.vertices()
	#inpt = input()
	#print ""
	#print ""
	#t1 = time.time()
	#g.extendedFloyd()
	#distanciaX, caminhoX = g.caminhoMinimoHospital(inpt)
	#t2 = time.time()
	#print "Tempo:"
	#print(t2-t1)
	#print "Ambulancia mais proxima da emergencia:"
	#print caminhoX[0]
	#print "Distancia:"
	#print distanciaX
	#print "Caminho:"
	#print caminhoX
	#print ""
	#print ""


