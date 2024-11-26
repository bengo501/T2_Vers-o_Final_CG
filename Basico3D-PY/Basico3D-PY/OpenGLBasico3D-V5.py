from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import numpy as np
from PIL import Image
import random
import math
import os

# Variáveis globais
Texturas = []
AnguloCanhao = 0.0  
PosX, PosZ = 0.0, 15.0  
AnguloVeiculo = 0.0  
Velocidade = 0.3
White = (1.0, 1.0, 1.0)  # Cor branca
projeteis = []  # Lista para armazenar projéteis
pontos = 0  # Pontuação total
shotStrength = 50  # Força do disparo
frameRate = 60  # Taxa de quadros
camera_mode = 1  # 1: livre, 2: veículo, 3: canhão, 4: isométrica, 5: cima, 6: lateral
g = 9.81  # Aceleração da gravidade

# Variáveis globais
AnguloArticulacao1 = 0.0  # Rotação no eixo Y
AnguloArticulacao2 = 0.0  # Inclinação do canhão
forca_disparo = 50  # Força inicial de lançamento
velocidade_veiculo = 2.5  # Velocidade do veículo (m/s)

# Variáveis globais para controle de inclinação e comprimento do ray cast
comprimento_ray = 15.0  # Comprimento do ray cast
angulo_vertical_ray = 0  # Inclinação vertical do ray cast

pontos = 0
amigos = []   # Lista de posições dos amigos
inimigos = [] # Lista de posições dos inimigos
largura_piso = 50
comprimento_piso = 25


def read_tri_file(file_path):
    #Lê um arquivo .tri e retorna uma lista de triângulos, onde cada triângulo é representado por 9 valores (3 vértices).
    triangles = []
    with open(file_path, 'r') as file:
        lines = file.readlines()
        for line in lines:
            vertices_str = line.strip().split()
            if len(vertices_str) >= 9:
                try:
                    vertices = list(map(float, vertices_str[:9]))
                    triangles.append(vertices)
                except ValueError as e:
                    print(f"Erro ao converter os valores '{line.strip()}': {e}")
            else:
                print(f"Erro: a linha '{line.strip()}' .")
    return triangles


class ObjetoTri:
    def __init__(self, file_path, object_type="neutral"):
        self.triangles = read_tri_file(file_path)
        self.object_type = object_type 

    def draw(self):
        if self.object_type == "vehicle":
            color = (0.5, 0.5, 0.5)  
        elif self.object_type == "friend":
            color = (0.0, 1.0, 0.0) 
        elif self.object_type == "enemy":
            color = (1.0, 0.0, 0.0)  
        else:
            color = (1.0, 1.0, 1.0)  

        desenhar_objeto_tri(self.triangles, color, self.object_type)
    
    def __init__(self, file_path):
        self.triangles = read_tri_file(file_path)

    def desenhar(self):
        glBegin(GL_TRIANGLES)
        for triangle in self.triangles:
            v1 = np.array([triangle[0], triangle[1], triangle[2]])
            v2 = np.array([triangle[3], triangle[4], triangle[5]])
            v3 = np.array([triangle[6], triangle[7], triangle[8]])
            
            # Calcular e definir a normal para o triângulo atual
            normal = calcular_normal(v1, v2, v3)
            glNormal3f(normal[0], normal[1], normal[2])
            
            # Desenhar o triângulo com flat shading
            glVertex3f(*v1)
            glVertex3f(*v2)
            glVertex3f(*v3)
        glEnd()


def aplicar_sombra():
    shadow_matrix = [
        1, 0, 0, 0,
        0, 0, 0, 0,  # Projeta no eixo Y
        0, 0, 1, 0,
        0, 0.01, 0, 1  # Ajuste para evitar o z-fighting com o chão
    ]
    glMultMatrixf(shadow_matrix)


def desenhar_objeto_com_sombra(objeto):
    glPushMatrix()
    glDisable(GL_LIGHTING)  # Desabilita iluminação para sombra
    glColor3f(0.1, 0.1, 0.1)  # Cor da sombra (escuro)
    aplicar_sombra()
    objeto.desenhar()  # Desenha a sombra
    glPopMatrix()

    # Desenhar o objeto real com iluminação ativada
    glEnable(GL_LIGHTING)
    objeto.desenhar()


# Classe Shot para projéteis que seguem o caminho do raycast
class Shot:
    def __init__(self, P0, B, C, timeIncrement):
        self.P0 = P0  # Ponto inicial
        self.P1 = B   # Ponto de controle intermediário
        self.P2 = C   # Ponto final
        self.timeIncrement = timeIncrement
        self.shotTime = 0.0
        self.hitTheGround = False
        self.tempo_de_vida = 3.0  # Tempo de vida em segundos

    def calcular_posicao(self):
        # Atualiza o tempo de animação
        self.shotTime += self.timeIncrement
        self.tempo_de_vida -= self.timeIncrement  # Diminui o tempo de vida

        if self.shotTime > 1.0:
            self.hitTheGround = True
            return (self.P2[0], 0, self.P2[2])  # Define o projétil no chão ao final

        oneMinusT = 1 - self.shotTime

        # Interpolação Bézier entre P0, P1 e P2
        R1 = (
            self.P0[0] * oneMinusT + self.P1[0] * self.shotTime,
            self.P0[1] * oneMinusT + self.P1[1] * self.shotTime,
            self.P0[2] * oneMinusT + self.P1[2] * self.shotTime
        )

        R2 = (
            self.P1[0] * oneMinusT + self.P2[0] * self.shotTime,
            self.P1[1] * oneMinusT + self.P2[1] * self.shotTime,
            self.P1[2] * oneMinusT + self.P2[2] * self.shotTime
        )

        return (
            R1[0] * oneMinusT + R2[0] * self.shotTime,
            R1[1] * oneMinusT + R2[1] * self.shotTime,
            R1[2] * oneMinusT + R2[2] * self.shotTime
        )

    def atualizar(self):
        # Atualiza a posição do projétil e retorna a nova posição
        return self.calcular_posicao()

    def tempo_expirado(self):
        # Verifica se o tempo de vida expirou
        return self.tempo_de_vida <= 0.0

    def hit_the_ground(self):
        # Retorna se o projétil atingiu o chão
        return self.hitTheGround



# =======================================================

# Listas globais para armazenar objetos amigos e inimigos
objetos_amigos = []
objetos_inimigos = []

# Função para carregar múltiplos arquivos .tri para amigos e inimigos usando read_tri_file
def carregarObjetos():
    #Carrega os modelos .tri para amigos e inimigos (amigo1.tri a amigo10.tri e inimigo1.tri a inimigo10.tri).
    global objetos_amigos, objetos_inimigos
    
    # Limpa as listas para evitar duplicação em caso de recarga
    objetos_amigos.clear()
    objetos_inimigos.clear()

    # Loop para carregar cada arquivo de amigo e inimigo
    try:
        for i in range(1, 11):
            # Monta o caminho para cada arquivo
            amigo_path = f"./assets/amigo{i}.tri"
            inimigo_path = f"./assets/inimigo{i}.tri"

            # Carrega os triângulos para cada amigo e inimigo
            amigo_triangles = read_tri_file(amigo_path)
            inimigo_triangles = read_tri_file(inimigo_path)

            # Adiciona os triângulos carregados às listas
            objetos_amigos.append(amigo_triangles)
            objetos_inimigos.append(inimigo_triangles)

            print(f"Amigo {i} e Inimigo {i} carregados com sucesso.")
    except Exception as e:
        print(f"Erro ao carregar objetos: {e}")

# =================================================================

amigo_posicoes = [
    (-10, 0, -10), (-15, 0, -15), (-12, 0, -12), (-14, 0, -14), (-4, 0, -9),
    (-4, 0, -4), (-6, 0, -6), (-6, 0, -10), (-9, 0, -9), (-9, 0, -15)
]
inimigo_posicoes = [
    (17, 0, -5), (6, 0, -7), (5, 0, -5), (8, 0, -10), (12, 0, -12),
    (12, 0, -8), (15, 0, -15), (10, 0, -15), (14, 0, -22), (16, 0, -19)
]

def desenhar_objeto_tri(triangles, color, object_type="neutral"):
    
    if object_type == "vehicle":
        glShadeModel(GL_FLAT)  
    else:
        glShadeModel(GL_SMOOTH)  


    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)

    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color)
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, [1.0, 1.0, 1.0, 1.0])
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 50.0)  

    glBegin(GL_TRIANGLES)
    for triangle in triangles:

        v0 = np.array(triangle[0:3])
        v1 = np.array(triangle[3:6])
        v2 = np.array(triangle[6:9])

        normal = np.cross(v1 - v0, v2 - v0)
        normal = normal / np.linalg.norm(normal)  

        glNormal3f(*normal)

        for j in range(0, len(triangle), 3):
            glVertex3f(triangle[j], triangle[j + 1], triangle[j + 2])
    glEnd()

    glDisable(GL_LIGHTING)
    glDisable(GL_LIGHT0)
    
    

# Função para desenhar os objetos carregados no display
def desenhar_objeto(triangles):
    #Renderiza o modelo .tri a partir de uma lista de triângulos.
    glBegin(GL_TRIANGLES)
    for triangle in triangles:
        for i in range(0, len(triangle), 3):
            glVertex3f(triangle[i], triangle[i + 1], triangle[i + 2])
    glEnd()

# Atualização da função desenhar_objetos para usar as novas listas carregadas
def desenhar_objetos():
    #Desenha objetos .tri para amigos e inimigos nas posições especificadas.
    # Desenha os amigos
    glColor3f(0, 1, 0)  # Verde para amigos
    for pos, amigo in zip(amigo_posicoes, objetos_amigos):
        glPushMatrix()
        glTranslatef(*pos)  # Posiciona o amigo
        glScalef(2, 2, 2)  # Ajusta o tamanho do modelo, se necessário
        desenhar_modelo_tri(amigo)  # Chama função para desenhar o modelo .tri
        glPopMatrix()

    # Desenha os inimigos
    glColor3f(1, 0, 0)  # Vermelho para inimigos
    for pos, inimigo in zip(inimigo_posicoes, objetos_inimigos):
        glPushMatrix()
        glTranslatef(*pos)  # Posiciona o inimigo
        glScalef(2, 2, 2)  # Ajusta o tamanho do modelo, se necessário
        desenhar_modelo_tri(inimigo)  # Chama função para desenhar o modelo .tri
        glPopMatrix()
# =================================================================

def ajustar_forca_disparo(incremento):
    global forca_disparo, AnguloArticulacao2
    # Ajusta a força de disparo dentro dos limites estabelecidos
    forca_disparo = max(min_forca_disparo, min(max_forca_disparo, forca_disparo + incremento))
    print(f"Força de disparo ajustada para: {forca_disparo}")

    # Relaciona a força de disparo à inclinação do canhão
    # O ângulo será proporcional ao valor de forca_disparo
    # Exemplo: 10 <= forca_disparo <= 100 -> -10° <= AnguloArticulacao2 <= 45°
    AnguloArticulacao2 = -10 + (forca_disparo - min_forca_disparo) * (45 - (-10)) / (max_forca_disparo - min_forca_disparo)
    print(f"Inclinação do canhão ajustada para: {AnguloArticulacao2}°")


# Função para lançar o projétil na direção do raycast
def atiraProjetil():
    global projeteis, AnguloArticulacao1, angulo_vertical_ray, PosX, PosZ, forca_disparo, frameRate

    # Define a origem e pontos de controle do projétil com base na curva do raycast
    ponta_canhao_x = PosX + 1.5 * math.sin(math.radians(-AnguloArticulacao1))
    ponta_canhao_y = 0.75 + 1.5 * math.sin(math.radians(-angulo_vertical_ray))
    ponta_canhao_z = PosZ - 1.5 * math.cos(math.radians(-AnguloArticulacao1))

    # Define o ponto intermediário e o ponto final (C) da curva Bézier
    B = (
        ponta_canhao_x + forca_disparo * 0.1 * math.sin(math.radians(-AnguloArticulacao1)),
        ponta_canhao_y + forca_disparo * 0.1 * math.sin(math.radians(-angulo_vertical_ray)),
        ponta_canhao_z - forca_disparo * 0.1 * math.cos(math.radians(-AnguloArticulacao1))
    )
    C = (
        B[0] + 5.0 * math.sin(math.radians(-AnguloArticulacao1)),
        max(0, ponta_canhao_y + forca_disparo * 0.2),
        B[2] - 5.0 * math.cos(math.radians(-AnguloArticulacao1))
    )

    # Calcula a duração e incremento de tempo do projétil
    defaultAnimateDuration = 0.4
    targetDuration = defaultAnimateDuration * (forca_disparo / 10)
    timeIncrement = targetDuration / frameRate

    # Adiciona um novo projétil seguindo a trajetória do raycast
    projeteis.append(Shot((ponta_canhao_x, ponta_canhao_y, ponta_canhao_z), B, C, timeIncrement))


# Função para desenhar o raycast usando uma curva Bézier
def desenhar_ray():
    global AnguloArticulacao1, AnguloArticulacao2, comprimento_ray, angulo_vertical_ray, forca_disparo

    # Define a altura com base na força do disparo
    ajuste_altura = forca_disparo * 0.1

    # Define o ponto inicial do ray cast, que é a ponta do canhão
    ponta_cano_x = PosX + math.sin(math.radians(AnguloVeiculo - AnguloArticulacao1)) * 1.5
    ponta_cano_y = 0.75
    ponta_cano_z = PosZ - math.cos(math.radians(AnguloVeiculo - AnguloArticulacao1)) * 1.5

    # Define o ponto de controle intermediário para a curva Bézier
    controle_x = ponta_cano_x + comprimento_ray * math.sin(math.radians(AnguloVeiculo - AnguloArticulacao1))
    controle_y = ponta_cano_y - ajuste_altura * math.sin(math.radians(angulo_vertical_ray + AnguloArticulacao2))
    controle_z = ponta_cano_z - comprimento_ray * math.cos(math.radians(AnguloVeiculo - AnguloArticulacao1))

    # Define o ponto final do ray cast, projetando-o para trás do canhão
    destino_x = controle_x + comprimento_ray * math.sin(math.radians(AnguloVeiculo - AnguloArticulacao1))
    destino_y = max(0, controle_y)  # Mantém o ponto final no chão ou acima
    destino_z = controle_z - comprimento_ray * math.cos(math.radians(AnguloVeiculo - AnguloArticulacao1))

    # Desenha a curva Bézier para o ray cast invertido
    glLineWidth(4.5)
    glColor3f(1.0, 0.0, 0.0)  # Ray cast em vermelho
    glBegin(GL_LINE_STRIP)

    # Calcula a curva Bézier e desenha
    for t in np.linspace(0, 1, 30):
        one_minus_t = 1 - t
        R1_x = one_minus_t * ponta_cano_x + t * controle_x
        R1_y = one_minus_t * ponta_cano_y + t * controle_y
        R1_z = one_minus_t * ponta_cano_z + t * controle_z

        R2_x = one_minus_t * controle_x + t * destino_x
        R2_y = one_minus_t * controle_y + t * destino_y
        R2_z = one_minus_t * controle_z + t * destino_z

        C1_x = one_minus_t * R1_x + t * R2_x
        C1_y = one_minus_t * R1_y + t * R2_y
        C1_z = one_minus_t * R1_z + t * R2_z

        glVertex3f(C1_x, C1_y, C1_z)

    glEnd()
    glLineWidth(1.6)


def imprimir_posicoes_paredao():
    largura_paredao = 40  # largura do paredão em unidades
    altura_paredao = 15   # altura do paredão em unidades
    inicio_x = -largura_paredao // 2  # posição inicial em X para centralizar o paredão
    posicao_z = 0  # profundidade do paredão, ajustável conforme o cenário

    print("Posições dos ladrilhos do paredão:")
    for y in range(altura_paredao):
        for x in range(largura_paredao):
            posicao_x = inicio_x + x  # calcula a posição X para cada ladrilho
            posicao_y = y  # posição Y depende da altura do ladrilho
            print(f"Ladrilho em posição (X: {posicao_x}, Y: {posicao_y}, Z: {posicao_z})")


# **********************************************************************
#  Funções auxiliares para cores
# **********************************************************************
def SetColor(color):
    glColor3f(color[0] / 255.0, color[1] / 255.0, color[2] / 255.0)

# **********************************************************************
#  Carrega Textura e Retorna o ID
# **********************************************************************
def LoadTexture(nome) -> int:
    try:
        image = Image.open(nome).transpose(Image.FLIP_TOP_BOTTOM)
        img_data = np.array(image, np.uint8)
        texture = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, texture)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.width, image.height, 0,
                     GL_RGB, GL_UNSIGNED_BYTE, img_data)
        print(f"Textura '{nome}' carregada com sucesso.")
        return texture
    except Exception as e:
        print(f"Erro ao carregar a textura '{nome}': {e}")
        return -1  # Retorna -1 se a textura não puder ser carregada

# **********************************************************************
#  Ativa ou Desativa Texturas
# **********************************************************************
def UseTexture(NroDaTextura: int):
    if NroDaTextura < 0 or NroDaTextura >= len(Texturas):
        glDisable(GL_TEXTURE_2D)
    else:
        glEnable(GL_TEXTURE_2D)
        glBindTexture(GL_TEXTURE_2D, Texturas[NroDaTextura])

# **********************************************************************
#  Desenha um Ladrilho com Borda e Preenchimento
# **********************************************************************
def DesenhaLadrilho(corBorda, corDentro):
    glBindTexture(GL_TEXTURE_2D, corDentro)  # Usa a textura passada
    glBegin(GL_QUADS)
    glNormal3f(0, 1, 0)
    glTexCoord2f(0.0, 0.0); glVertex3f(-0.5, 0.0, -0.5)
    glTexCoord2f(1.0, 0.0); glVertex3f(0.5, 0.0, -0.5)
    glTexCoord2f(1.0, 1.0); glVertex3f(0.5, 0.0, 0.5)
    glTexCoord2f(0.0, 1.0); glVertex3f(-0.5, 0.0, 0.5)
    glEnd()

# **********************************************************************
# Passa por parametro o índice da textura desejada.
# **********************************************************************
def DesenhaPiso(textura):
    UseTexture(0)  # Ativa a textura do piso
    glPushMatrix()
    glTranslated(-20, -1, 20)  # Posição inicial do piso
    glRotatef(90, 0, 1, 0)  # Rotaciona o piso em 90 graus no eixo Y
    glColor3f(1.0, 1.0, 1.0)  # Define a cor como branca
    glScalef(1.0, 1.0, 1.28)  # Aumenta o tamanho do ladrilho (escala)


    for x in range(-30, 31):  # Aumenta o intervalo de -30 a 30
        glPushMatrix()
        for z in range(-15, 16):  # Aumenta o intervalo de -15 a 15
            glPushMatrix()  # Adiciona um novo PushMatrix para aplicar escalas
            glScalef(2.0, 1.0, 2.0)  # Aumenta o tamanho do ladrilho (escala)

            # Aplique a rotação se necessário
            glRotatef(180, 1, 0, 0)  # Rotaciona o ladrilho em 180 graus no eixo X            
            DesenhaLadrilho(White, 1)  # Chama a função para desenhar o ladrilho
            glPopMatrix()  # Restaura a matriz anterior após desenhar o ladrilho
            glTranslated(0, 0, 1)  # Translada para o próximo ladrilho
        glPopMatrix()
        glTranslated(1, 0, 0)  # Move para a próxima coluna do piso
    glPopMatrix()
    UseTexture(-1)  # Desativa o uso de texturas após desenhar


# **********************************************************************
#  Desenha o Paredão com Ladrilhos Horizontais
# **********************************************************************
def DesenhaLadrilhoComTextura(celula_x, celula_y, max_x=40, max_y=15):
    #Desenha um ladrilho e aplica uma parte da textura com base na sua posição.
    # Calcula as coordenadas de textura para a célula atual
    tex_x_min = celula_x / max_x  # Coordenada mínima em X
    tex_x_max = (celula_x + 1) / max_x  # Coordenada máxima em X
    tex_y_min = celula_y / max_y  # Coordenada mínima em Y
    tex_y_max = (celula_y + 1) / max_y  # Coordenada máxima em Y

    glBegin(GL_QUADS)
    glNormal3f(0, 1, 0)  # Normal para o plano XY

    # Aplica coordenadas de textura em cada vértice
    glTexCoord2f(tex_x_min, tex_y_min)
    glVertex3f(-0.5, -0.5, 0)

    glTexCoord2f(tex_x_max, tex_y_min)
    glVertex3f(0.5, -0.5, 0)

    glTexCoord2f(tex_x_max, tex_y_max)
    glVertex3f(0.5, 0.5, 0)

    glTexCoord2f(tex_x_min, tex_y_max)
    glVertex3f(-0.5, 0.5, 0)
    glEnd()



# Função para posicionar amigos e inimigos manualmente atrás do paredão
def posicionar_amigos_e_inimigos():
    #Posiciona 10 amigos e 10 inimigos manualmente atrás do paredão, dentro da área do piso.
    global amigos, inimigos
    # Coordenadas fixas para os amigos (verde) - lado esquerdo atrás do paredão
    amigos = [
        (-40, 0, -10), (-43, 0, -8), (-51, 0, -6), (-59, 0, -4), (-67, 0, -2),
        (-42, 0, 0), (-63, 0, 2), (-61, 0, 4), (-31, 0, 6), (-37, 0, 8)
    ]
    
    # Coordenadas fixas para os inimigos (vermelho) - lado direito atrás do paredão
    inimigos = [
        (5, 0, -10), (7, 0, -8), (9, 0, -6), (11, 0, -4), (13, 0, -2),
        (15, 0, 0), (17, 0, 2), (19, 0, 4), (21, 0, 6), (23, 0, 8)
    ]


# Função para desenhar amigos e inimigos
def desenhar_amigos_inimigos():
    #Desenha os amigos e inimigos na cena.
    glColor3f(0, 1, 0)  # Verde para amigos
    for amigo in amigos:
        glPushMatrix()
        glTranslatef(*amigo)
        glScalef(2, 2, 2)  # Aumenta o tamanho do cubo
        glutSolidCube(1)
        glPopMatrix()
        
    glColor3f(1, 0, 0)  # Vermelho para inimigos
    for inimigo in inimigos:
        glPushMatrix()
        glTranslatef(*inimigo)
        glScalef(2, 2, 2)  # Aumenta o tamanho do cubo
        glutSolidCube(1)
        glPopMatrix()


# Estado do paredão: matriz 40x15 (cada valor representa uma célula ativa ou destruída)
estado_paredao = [[True for _ in range(15)] for _ in range(40)]

# Função para remover ladrilho do paredão após colisão
def remover_ladrilho(x, y):
    if 0 <= x < 40 and 0 <= y < 15:
        estado_paredao[x][y] = False

# =================================================================
def criar_paredao():
    #Cria o paredão e inicializa os ladrilhos.
    global paredao_ladrilhos

    for y in range(15):  # Altura do paredão
        for x in range(40):  # Largura do paredão
            ladrilho = {
                "pos": (x - 20, y, 0),  # Posição do ladrilho
                "ativo": True  # Define se o ladrilho está ativo ou não
            }
            paredao_ladrilhos.append(ladrilho)
# =================================================================
paredao_ladrilhos = []  # Lista de ladrilhos ativos da parede

def DesenhaParedao():
    #Desenha apenas os ladrilhos ativos do paredão com texturas específicas.
    UseTexture(1)  # Ativa a textura do paredão (bricks)

    for x in range(40):
        for y in range(15):
            if estado_paredao[x][y]:  # Verifica se o ladrilho está ativo
                glPushMatrix()
                glTranslatef(x - 20, y, 0)  # Posiciona o ladrilho corretamente
                glRotatef(270, 0, 0, 0)  # Rotaciona o ladrilho para o plano XY
                DesenhaLadrilhoComTextura(x, y)  # Desenha o ladrilho com a textura apropriada
                glPopMatrix()

    UseTexture(-1)  # Desativa a textura após desenhar


# **********************************************************************
#  Desenha o Veículo com Canhão Articulado
# **********************************************************************
def DesenhaVeiculo():
    #Desenha o veículo com articulações e canhão invertido, aplicando cor e sombreamento.
    global AnguloArticulacao1, AnguloArticulacao2

    # Habilitar iluminação para sombreamento
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)

    # Material da base do veículo (cor azul)
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, [0.0, 0.0, 1.0, 1.0])  # Azul
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, [0.5, 0.5, 0.5, 1.0])
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 30.0)

    # Desenhar a base do veículo (paralelepípedo)
    glPushMatrix()
    glScalef(3.0, 1.0, 2.0)  # Base com comprimento 3m e largura 2m
    glutSolidCube(1.0)
    glPopMatrix()

    # Articulação 1: Rotação no eixo Y (base girando)
    glPushMatrix()
    glTranslatef(0.0, 0.5, 0.0)  # Eleva para a base da articulação
    glRotatef(AnguloArticulacao1, 0, 1, 0)  # Rotação no eixo Y

    # Material do canhão (cor vermelha)
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, [1.0, 0.0, 0.0, 1.0])  # Vermelho
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, [0.5, 0.5, 0.5, 1.0])
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 50.0)

    # Desenhar o segmento do canhão
    glPushMatrix()
    glTranslatef(0.0, 0.5, -1.0)  # Move o canhão para trás em vez de para frente
    glRotatef(AnguloArticulacao2, 1, 0, 0)  # Rotação no eixo X (inclinação)
    glScalef(0.5, 0.5, -2.0)  
    glutSolidCube(1.0)
    glPopMatrix()

    glPopMatrix()

    # Desativar iluminação após renderizar
    glDisable(GL_LIGHTING)

# **********************************************************************
#  Configurações Iniciais
# **********************************************************************
def init():
    global Texturas
    glClearColor(0.6, 0.8, 1.0, 1.0)  # Fundo azul claro
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_CULL_FACE)
    glDisable(GL_COLOR_MATERIAL)
    glEnable(GL_NORMALIZE)
    
    # Habilitar iluminação e definir o modo flat shading
    glEnable(GL_LIGHTING)
    glShadeModel(GL_FLAT)  # Define o sombreamento para flat shading
    
    # Configuração da fonte de luz
    glEnable(GL_LIGHT0)
    light_pos = [4.0, 10.0, 6.0, 1.0]  # Posição da luz
    light_ambient = [0.3, 0.3, 0.3, 1.0]  # Luz ambiente
    light_diffuse = [0.8, 0.8, 0.8, 1.0]  # Luz difusa
    light_specular = [1.0, 1.0, 1.0, 1.0]  # Luz especular

    glLightfv(GL_LIGHT0, GL_POSITION, light_pos)
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient)
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse)
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular)

    # Configuração de material padrão (para garantir reflexão correta da luz)
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, [0.5, 0.5, 0.5, 1.0])
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, [1.0, 1.0, 1.0, 1.0])
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 50.0)
    
    # Carregar texturas
    piso_texture = LoadTexture("./grama.jpg")
    bricks_texture = LoadTexture("./tijolos.jpg")
    Texturas.extend([piso_texture, bricks_texture])
    
    # Posicionar amigos e inimigos
    posicionar_amigos_e_inimigos()


def calcular_normal(v1, v2, v3):
    # Vetores da face
    U = np.subtract(v2, v1)
    V = np.subtract(v3, v1)

    # Produto cruzado para calcular a normal
    normal = np.cross(U, V)
    normal = normal / np.linalg.norm(normal)  # Normalização
    return normal


def verificar_colisao(projétil):
    #Verifica se o projétil colidiu com o paredão.
    x = int(projétil.P2[0] + 20)  # Ajusta coordenadas para índices do paredão
    y = int(projétil.P2[1])
    if 0 <= x < 40 and 0 <= y < 15 and estado_paredao[x][y]:
        remover_ladrilho(x, y)  # Remove ladrilho do paredão
        return True
    return False

# =================================================================
def verificar_colisao_com_objeto(projetil, objetos, pontos_valor):
    #Verifica se o projétil colidiu com algum objeto (amigo ou inimigo) e atualiza a pontuação.
    global pontos
    for obj in objetos[:]:
        for vertice in obj:  # Verifica a colisão com cada vértice do objeto
            if abs(projetil.P2[0] - vertice[0]) < 1.5 and abs(projetil.P2[2] - vertice[2]) < 1.5:
                objetos.remove(obj)  # Remove o objeto atingido
                pontos += pontos_valor  # Atualiza pontuação (positivo ou negativo)
                tipo = "Amigo" if pontos_valor < 0 else "Inimigo"
                print(f"{tipo} atingido! Pontuação: {pontos}")
                return True
    return False

def verificar_colisao_com_amigo(projetil):
    #Verifica se o projétil colidiu com algum amigo .tri e atualiza a pontuação.
    return verificar_colisao_com_objeto(projetil, objetos_amigos, -10)

def verificar_colisao_com_inimigo(projetil):
    #Verifica se o projétil colidiu com algum inimigo .tri e atualiza a pontuação.
    return verificar_colisao_com_objeto(projetil, objetos_inimigos, 10)

#================================================================
def verificar_colisao_com_canhao(projetil):
    #Verifica se o projétil colidiu com o canhão.
    global PosX, PosZ

    # Verifica colisão em X, Y e Z com base na posição do canhão
    if (abs(projetil.P2[0] - PosX) < 1.0 and  # Checa proximidade em X
        abs(projetil.P2[1]) < 1.0 and         # Checa proximidade em Y (Altura)
        abs(projetil.P2[2] - PosZ) < 1.0):    # Checa proximidade em Z
        print("Canhão atingido! Fim do jogo.")
        os._exit(0)  # Encerra o jogo imediatamente
        return True
    return False

# =================================================================

def verificar_colisao_com_chao(projetil):
    #Verifica se o projétil atingiu o chão e penaliza a pontuação.
    global pontos

    if projetil.hit_the_ground():  # Verifica se o projétil chegou ao chão
        pontos -= 5  # Penaliza o jogador em 5 pontos
        print(f"Projétil atingiu o chão! Pontuação: {pontos}")
        return True
    return False

# =================================================================
def remover_vizinhos(x, y):
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < 40 and 0 <= ny < 15:
                estado_paredao[nx][ny] = False

# =================================================================
def verificar_colisao_com_parede(projétil):
    #Verifica se o projétil colidiu com algum ladrilho da parede e contabiliza pontos.
    global pontos

    # Ajusta as coordenadas do projétil para o sistema de referência do paredão
    x = int(round(projétil.P2[0] + 20))  # Arredonda para garantir coordenadas inteiras
    y = int(round(projétil.P2[1]))  # Arredonda para garantir coordenadas inteiras

    # Verifica se a colisão é válida
    if 0 <= x < 40 and 0 <= y < 15 and estado_paredao[x][y]:
        remover_vizinhos(x, y)  # Remove o ladrilho e seus vizinhos
        pontos += 5  # Incrementa pontuação
        print(f"Ladrilho atingido em ({x}, {y}). Pontuação: {pontos}")

        # Força a atualização do display após a remoção dos ladrilhos
        glutPostRedisplay()
        return True
    return False


# **********************************************************************
#  Display
# **********************************************************************
def display():
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

    # Define a posição da câmera com diferentes modos
    if camera_mode == 1:  # Câmera livre (mais distante)
        gluLookAt(0, 15, 40, 0, 0, 0, 0, 1, 0)
    elif camera_mode == 2:  # Câmera do veículo
        gluLookAt(PosX, 5, PosZ + 15, PosX, 0, PosZ, 0, 1, 0)
    elif camera_mode == 3:  # Câmera do canhão
        cannonPosition = (PosX, 0, PosZ)
        B = (cannonPosition[0] + 1 * math.sin(math.radians(AnguloCanhao)), 
             cannonPosition[1] + 5, 
             cannonPosition[2] - 1 * math.cos(math.radians(AnguloCanhao)))
        gluLookAt(B[0], B[1], B[2],  # Posição da câmera
                  cannonPosition[0], cannonPosition[1], cannonPosition[2],  # Ponto de olhar
                  0, 1, 0)  # Vetor up
    elif camera_mode == 4:  # Câmera isométrica mais afastada
        gluLookAt(30, 30, 30, 0, 0, 0, 0, 1, 0)
    elif camera_mode == 5:  # Câmera de cima
        gluLookAt(0, 40, 0, 0, 0, 0, 0, 0, -1)
    elif camera_mode == 6:  # Câmera lateral direita
        gluLookAt(30, 5, PosZ, 0, 0, PosZ, 0, 1, 0)
    elif camera_mode == 7:  # Câmera traseira
        gluLookAt(PosX, 5, PosZ - 15, PosX, 0, PosZ, 0, 1, 0)
    elif camera_mode == 8:  # Câmera lateral esquerda
        gluLookAt(-30, 5, PosZ, 0, 0, PosZ, 0, 1, 0)

        
    # Desenha o piso e o paredão
    DesenhaPiso(Texturas[0])
    DesenhaParedao()

    # Desenha amigos e inimigos posicionados conforme as posições definidas e com escala reduzida
    for pos, amigo in zip(amigo_posicoes, objetos_amigos):
        glPushMatrix()
        glTranslatef(*pos)  # Ajusta posição de cada amigo conforme amigo_posicoes
        glRotatef(270, 0, 1, 0)  # Rotaciona o amigo para que fique de frente para o canhão
        glScalef(0.75, 0.75, 0.75)  # Diminui o tamanho do modelo para 50%
        desenhar_objeto_tri(amigo, (0, 1, 0))  # Cor verde para amigos
        glPopMatrix()

    for pos, inimigo in zip(inimigo_posicoes, objetos_inimigos):
        glPushMatrix()
        glTranslatef(*pos)  # Ajusta posição de cada inimigo conforme inimigo_posicoes
        glRotatef(270, 0, 1, 0)  # Rotaciona o inimigo para que fique de frente para o canhão
        glScalef(0.75, 0.75, 0.75)  # Diminui o tamanho do modelo para 50%
        desenhar_objeto_tri(inimigo, (1, 0, 0))  # Cor vermelha para inimigos
        glPopMatrix()


    # Atualiza e desenha projéteis
    for projétil in projeteis[:]:
        nova_pos = projétil.atualizar()
        glPushMatrix()
        glTranslatef(*nova_pos)
        glutSolidSphere(0.4, 16, 16)  # Desenha o projétil
        glPopMatrix()

        
        # Verifique colisões e tempo de vida do projétil
        if (verificar_colisao_com_canhao(projétil) or
            verificar_colisao_com_amigo(projétil) or
            verificar_colisao_com_inimigo(projétil) or
            verificar_colisao_com_parede(projétil) or
            verificar_colisao_com_chao(projétil) or
            projétil.tempo_expirado()):  
            
            projeteis.remove(projétil)  # Remove o projétil após 2 segundos ou colisão
            glutPostRedisplay()  # Força o redesenho da tela

    # Desenha o veículo
    glPushMatrix()
    glTranslatef(PosX, 0, PosZ)
    glRotatef(AnguloVeiculo, 0, 1, 0)
    DesenhaVeiculo()
    glPopMatrix()

    # Desenha o raycast partindo da ponta do canhão
    desenhar_ray()

    # Troca de buffers para exibir o frame
    glutSwapBuffers()


# **********************************************************************
#  Redimensionamento da Tela
# **********************************************************************
def reshape(w, h):
    if h == 0:
        h = 1
    aspect_ratio = w / h
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(60.0, aspect_ratio, 0.1, 100.0)
    glViewport(0, 0, w, h)

# **********************************************************************
#  Animação e Controle de Teclado
# **********************************************************************
def animate():
    glutPostRedisplay()

def keyboard(key, x, y):
    global AnguloArticulacao1, AnguloArticulacao2, forca_disparo
    global PosX, PosZ, AnguloVeiculo, AnguloCanhao, camera_mode, comprimento_ray, angulo_vertical_ray

    if key == b'\x1b':  # ESC para sair
        os._exit(0)

    # Alternar modos de câmera com teclas de 1 a 8
    elif key == b'1':  # Câmera livre
        camera_mode = 1
        print("Câmera: Livre")
    elif key == b'2':  # Câmera do veículo
        camera_mode = 2
        print("Câmera: Veículo")
    elif key == b'3':  # Câmera do canhão
        camera_mode = 3
        print("Câmera: Canhão")
    elif key == b'4':  # Câmera isométrica (mais afastada)
        camera_mode = 4
        print("Câmera: Isométrica")
    elif key == b'5':  # Câmera de cima
        camera_mode = 5
        print("Câmera: Vista superior")
    elif key == b'6':  # Câmera lateral direita
        camera_mode = 6
        print("Câmera: Lateral direita")
    elif key == b'7':  # Câmera traseira
        camera_mode = 7
        print("Câmera: Traseira")
    elif key == b'8':  # Câmera lateral esquerda
        camera_mode = 8
        print("Câmera: Lateral esquerda")

    # Controles das articulações do veículo e canhão
    elif key == b'A':  # Gira articulação 1 no sentido horário
        AnguloArticulacao1 -= 5
    elif key == b'a':  # Gira articulação 1 no sentido anti-horário
        AnguloArticulacao1 += 5
    elif key == b'B':  # Inclina articulação 2 para frente
        AnguloArticulacao2 += 5
    elif key == b'b':  # Inclina articulação 2 para trás
        AnguloArticulacao2 -= 5

    # Ajuste da força de disparo
    elif key == b'+':
        forca_disparo = min(forca_disparo + 5, 100)  # Limite máximo de 100
        AnguloArticulacao2 -= 5
        print(f"Força de disparo: {forca_disparo}")
        angulo_vertical_ray = max(angulo_vertical_ray - 5, -10)
        print(f"Inclinação do canhão: {angulo_vertical_ray}°")
        comprimento_ray = max(comprimento_ray - 2.0, 5.0)
        print(f"Comprimento do ray cast: {comprimento_ray}")
        
    elif key == b'-':
        forca_disparo = max(forca_disparo - 5, 10)  # Limite mínimo de 10
        AnguloArticulacao2 += 5
        print(f"Força de disparo: {forca_disparo}")
        angulo_vertical_ray = min(angulo_vertical_ray + 5, 45)
        print(f"Inclinação do canhão: {angulo_vertical_ray}°")
        comprimento_ray = min(comprimento_ray + 2.0, 20.0)
        print(f"Comprimento do ray cast: {comprimento_ray}")
        

    # Movimentação do veículo
    elif key == b'w':  # Move o veículo para frente
        PosX += math.sin(math.radians(AnguloVeiculo)) * 2.5 / 60  # Velocidade ajustada
        PosZ -= math.cos(math.radians(AnguloVeiculo)) * 2.5 / 60
    elif key == b's':  # Move o veículo para trás
        PosX -= math.sin(math.radians(AnguloVeiculo)) * 2.5 / 60
        PosZ += math.cos(math.radians(AnguloVeiculo)) * 2.5 / 60
    elif key == b'a':  # Rotaciona o veículo para a esquerda
        AnguloVeiculo += 5
    elif key == b'd':  # Rotaciona o veículo para a direita
        AnguloVeiculo -= 5

    # Controles do canhão (inclinação vertical e horizontal)
    elif key == b'r':  # Inclina o canhão para cima
        angulo_vertical_ray = min(angulo_vertical_ray + 5, 45)
        print(f"Inclinação do canhão: {angulo_vertical_ray}°")
    elif key == b'f':  # Inclina o canhão para baixo
        angulo_vertical_ray = max(angulo_vertical_ray - 5, -10)
        print(f"Inclinação do canhão: {angulo_vertical_ray}°")
    elif key == b'e':  # Aumenta o comprimento do ray cast (alcance)
        comprimento_ray = min(comprimento_ray + 2.0, 20.0)
        print(f"Comprimento do ray cast: {comprimento_ray}")
    elif key == b'q':  # Diminui o comprimento do ray cast
        comprimento_ray = max(comprimento_ray - 2.0, 5.0)
        print(f"Comprimento do ray cast: {comprimento_ray}")

    # Disparo de projétil com a tecla espaço
    elif key == b' ':
        atiraProjetil()  # Lança um novo projétil

    # Atualiza o display após qualquer ação
    glutPostRedisplay()




# **********************************************************************
#  Função Principal
# **********************************************************************
glutInit()
glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
glutInitWindowSize(800, 600)
glutCreateWindow("Veículo com Piso e Paredão")
init()
criar_paredao()  # Cria os ladrilhos do paredão
carregarObjetos()  # Carrega os objetos do cenário
#imprimir_posicoes_paredao()
glutDisplayFunc(display)
glutReshapeFunc(reshape)
glutIdleFunc(animate)
glutKeyboardFunc(keyboard)

glutMainLoop()