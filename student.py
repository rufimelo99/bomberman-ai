import sys
import json
import asyncio
import websockets
import getpass
import os
import random
import math
from mapa import Map

from random import randrange




class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


async def agent_loop(server_address="localhost:8000", agent_name="student"):
    async with websockets.connect(f"ws://{server_address}/player") as websocket:

        # Receive information about static game properties
        await websocket.send(json.dumps({"cmd": "join", "name": agent_name}))
        msg = await websocket.recv()
        game_properties = json.loads(msg)

        # You can create your own map representation or use the game representation:
        mapa = Map(size=game_properties["size"], mapa=game_properties["map"])
        #print(game_properties["map"])


        
        
        movements_array_to_safety = []
        powerups_obtained=[]
        #array of obligatory moves that the agent must do in order to kill enemies on corners
        obligatory_moves=[]
        
        position_to_explode_bomb_Balloom = [[4,1],[1,15],[12,1]]
        tries_to_kill=0
        got_powerup=False #see if has powerup from level
        

        safe_bomb = False
        nivel = 0
        lives = 3
        
        

        while True:
            try:

                state = json.loads(
                    await websocket.recv()
                )  
                print(len(websocket.messages))

                if state["lives"] != lives:
                    lives = state["lives"]
                    movements_array_to_safety = []
                    obligatory_moves = []
                    safe_bomb = False

                    tries_to_kill = 0
                    position_to_explode_bomb_Balloom = [[4,1],[1,15],[12,1]]
                    

            
                # LOAD INFORMATION   OF THE STATE
                enemies, walls, bombs, powerups, bonus, saida, bomberman = get_info(state)
                
                enemies_pos =  get_enemies_list(state)
                #print(enemies)
                map_info = game_properties["map"]

                # Atualizar informação do mapa ao passar de nível
                if(nivel < state["level"]):
                    nivel = state["level"]
                    mapa.walls = state["walls"]
                    lives = state["lives"]
                    movements_array_to_safety = []
                    obligatory_moves = []
                    safe_bomb = False
                    tries_to_kill = 0
                    position_to_explode_bomb_Balloom = [[4,1],[1,15],[12,1]]
                    got_powerup=False



                # inicializar key
                key = ""
               
                # nearest enemy informationsource venv/bin/activate
                if enemies != []:
                    near_enemy, enemy_dis = find_near_enemy(enemies, bomberman)
                elif enemies == []:
                    enemy_dis = 100
                print(enemy_dis, near_enemy)
                # closer wall information
                if walls != []:
                    closer, wall_dis = find_closer_wall(bomberman, walls)
                else:
                    wall_dis = 9999

                # Caso existam bombas no mapa
                if exists_bomb(bombs):
                    #se ja tiver sido atribuido movimentos para fugir
                    if movements_array_to_safety != []:
                        key, safe_bomb, movements_array_to_safety = complete_safety_process(movements_array_to_safety, bomberman, safe_bomb)
                            

                    else:
                        # Se o bomberman nao estiver num posição segura (fugir da bomba)
                        if not safe_bomb:
                            info, explosion_range = bomb_info(bombs, mapa)
                            
                            #reduzir distanica dos safe place da explosao em relacao aos inimigos
                            security_distance=10
                            for i in range(security_distance):
                                movements_array_to_safety = bomb_run_v2(info, bomberman, mapa, near_enemy, walls, movements_array_to_safety, enemies, map_info, security_distance-i)
                                #print("security_distabce: ", security_distance-i)
                                if movements_array_to_safety != []:
                                    break
                            if movements_array_to_safety != []:            
                                key = go_to_next_block(movements_array_to_safety.pop(0), bomberman)
                                #retirar movimento da lista
                                
                            
       
                        # Caso o bomberman esteja numa posição segura (aguardar parado até bomba explodir)
                        else:
                            key = ""
                            if has_detonator(powerups_obtained):
                                key="A"

                # Caso nao existam bombas no mapa
                else:
                    #print("THERE ARE NO BOMBS")
                    # Quando a bomba explode estamos safe
                    if safe_bomb == True:
                        safe_bomb = False

                    # Há Algum Powerup no mapa ? (YES - GET THE POWERUP)
                    if powerups != []:
                        coord = powerups[0][0]
                        #apenas para prevenir que encontre um inimigo ao apanhar um powerup
                        if enemies != []:
                            #ha inimigos eo powerup ta acessivel
                            near_enemy_from_powerup, enemy_dis_from_powerup = find_near_enemy(enemies, coord)
                            if enemy_dis_from_powerup > 5:
                                #safe to grab powerup
                                key, powerups_obtained, got_powerup = grab_powerup(map_info, state, bomberman, coord, walls, enemies_pos, powerups_obtained, got_powerup)
                            else:
                                #inimios demasiado perto para grab powerup
                                if walls != []:
                                    wall_to_destroy=closer
                                    if is_possible_to_destroy_wall(map_info, bomberman, wall_to_destroy, walls, enemies_pos):
                                        key, tries_to_kill = destroy_wall(map_info, bomberman, wall_to_destroy, tries_to_kill, walls, enemies_pos)

                        else:
                            key, powerups_obtained, got_powerup = grab_powerup(map_info, state, bomberman, coord, walls, enemies_pos, powerups_obtained, got_powerup)



                    # Não há PowerUps no mapa   
                    else:

                        if enemy_dis < get_safe_dis(near_enemy):
                            key = "B"
                        else:
                            if enemies != []:
                                if obligatory_moves != []:
                                    key, obligatory_moves = use_obligatory_moves(obligatory_moves, bomberman)
                                else:
                                    #há algum inimigo que valha matar primeiro?
                                    if enemy_dis < wall_dis and not (near_enemy['name']=='Balloom' or near_enemy['name']=='Doll') and tries_to_kill<game_properties['size'][0]:
                                        key, obligatory_moves = kill_smarter_enemies(game_properties, mapa, map_info, bomberman, enemies, walls, near_enemy, enemy_dis, obligatory_moves)

                                    else:
                                        # Há parades no mapa? 
                                        if walls != []:
                                            if is_possible_to_destroy_wall(map_info, bomberman, closer, walls, enemies_pos):
                                                key, tries_to_kill = destroy_wall(map_info, bomberman, closer, tries_to_kill, walls, enemies_pos)

                                        #nao ha paredes, mas ainda ha inimigos
                                        elif walls == []:
                                            if exists_Dumb_Enemies(enemies):
                                                key, position_to_explode_bomb_Balloom = basic_method_kill_dumb_enemies(map_info, bomberman,enemies, enemies_pos, position_to_explode_bomb_Balloom, walls)
                                            else:
                                                key, obligatory_moves = kill_smarter_enemies(game_properties, mapa, map_info, bomberman, enemies, walls, near_enemy, enemy_dis, obligatory_moves)




                            # Se os inimigos morrerem todos
                            else:
                                #print("nao ha inimigos")
                                if saida != [] and got_powerup==True:
                                    #desclocar-se para a saida
                                    key = move_to_exit(map_info, bomberman, saida, walls)

                                else:
                                    #partir paredes para descobrir saida
                                    wall_to_destroy=closer
                                    if saida != []:
                                        #find exit
                                        key, tries_to_kill  = destroy_wall(map_info, bomberman, wall_to_destroy, tries_to_kill, walls, enemies_pos+[saida])

                                    else:
                                        #find powerup first
                                        key, tries_to_kill = destroy_wall(map_info, bomberman, wall_to_destroy, tries_to_kill, walls, enemies_pos)  
                               
                                       
                # Enviar a key para o servidor
                await websocket.send(
                        json.dumps({"cmd": "key", "key": key})
                )
                
            except websockets.exceptions.ConnectionClosedOK:
                print("Server has cleanly disconnected us")
                return

          


def get_safe_dis(near_enemy):
    dis = 0
    if near_enemy['name'] == 'Balloom':
        dis = 3
    if near_enemy['name'] == 'Oneal':
        dis = 1
    if near_enemy['name'] == 'Doll':
        dis = 2
    if near_enemy['name'] == 'Minvo':
        dis = 1
    if near_enemy['name'] == 'Kondoria':
        dis = 1
    if near_enemy['name'] == 'Ovapi':
        dis = 2
    if near_enemy['name'] == 'Pass':
        dis = 2
    return dis
#main actions
def move_to_exit(map_info, bomberman, saida, walls):
    key=""
    next_move = astar(map_info, (bomberman[0], bomberman[1]), (saida[0], saida[1]), walls)
    if next_move != None:
	    if len(next_move)>1:
	        key = go_to_next_block(next_move[1], bomberman)
    return key

def complete_safety_process(movements_array_to_safety, bomberman, safe_bomb):
    key=""
    key = go_to_next_block(movements_array_to_safety[0], bomberman)
    movements_array_to_safety.pop(0)
    
    #se chegou ao bloco safe da explosao
    if len(movements_array_to_safety) == 0:
        safe_bomb=True
    return key, safe_bomb, movements_array_to_safety

def destroy_wall(map_info, bomberman, wall_to_destroy, tries_to_kill, walls, enemies_pos):
    key=""
    temp_walls=walls
    
    if wall_to_destroy in temp_walls:
        temp_walls.remove(wall_to_destroy)

    next_move = astar(map_info, (bomberman[0], bomberman[1]), (wall_to_destroy[0], wall_to_destroy[1]), enemies_pos+temp_walls)
    
    if next_move != None and len(next_move)>0:
        if len(next_move)>2:
            key = go_to_next_block(next_move[1], bomberman)
        else:
            key = "B"
            tries_to_kill = 0
    return key, tries_to_kill

def basic_method_kill_dumb_enemies(map_info, bomberman,enemies, enemies_pos, position_to_explode_bomb_Balloom, walls):
    key=""
    #alternativas para matar os balloom
    alternativa_encontrada = False
    for i in position_to_explode_bomb_Balloom:
        near_enemy, enemy_dis = find_near_enemy(enemies, i)
        if calculateDistance(i, near_enemy["pos"]) > 2:
            next_move = astar(map_info, (bomberman[0], bomberman[1]), (i[0], i[1]), enemies_pos+walls)
            alternativa_encontrada = True
            #print("astar path: {}".format(next_move))
            if next_move != None:
                if len(next_move)>1:
                    key = go_to_next_block(next_move[1], bomberman)
                else:
                    key = "B"
                    position_to_explode_bomb_Balloom = [i]
            
            break
    if alternativa_encontrada == False:
        #print("nao ha alternativa")
        key = ""
    return key, position_to_explode_bomb_Balloom

def grab_powerup(map_info, state, bomberman, coord, walls, enemies_pos, powerups_obtained, got_powerup):
    key = ""
    next_move = astar(map_info, (bomberman[0], bomberman[1]), (coord[0], coord[1]), walls+enemies_pos)
    #TO DO preciso de condicao correta para adicionar o powerup
    if next_move != None:
	    if len(next_move)>2 :
	        powerups_obtained.append(state['powerups'][0][1])
	        got_powerup=True
	    if len(next_move)>1:
	        key = go_to_next_block(next_move[1], bomberman)

    return key, powerups_obtained, got_powerup                    

def use_obligatory_moves(obligatory_moves, bomberman):
    key=""
    if len(obligatory_moves)>1:
        key = go_to_next_block(obligatory_moves[1], bomberman)
        obligatory_moves.pop(0)
    else:
        #reset array... it should be already enough to push enemy away from the corner
        obligatory_moves = []
    return key, obligatory_moves
def kill_smarter_enemies(game_properties, mapa, map_info, bomberman, enemies, walls, near_enemy, enemy_dis, obligatory_moves):
    key=""
    pos_near = is_enemy_on_corner(game_properties, near_enemy, bomberman)
    
    if pos_near != None and calculateDistance(bomberman, near_enemy["pos"]) < 3 and (near_enemy['name']=='Oneal' or near_enemy['name']=='Minvo' or near_enemy['name']=='Ovapi') :
        if bomberman==pos_near:
            #assustar o inimigo e fazelo fugir do canto
            key = ""
        else:
            next_move = astar(map_info, (bomberman[0], bomberman[1]), (pos_near[0], pos_near[1]), walls)
            if next_move != None:
	            for i in next_move:
	                obligatory_moves.append(i)
    else:
        if is_enemy_surrounded_except_by_one_block(mapa, near_enemy, walls):
            enemies_to_avoid_pos = []
            for i in enemies:
                if i['id']!=near_enemy['id']:
                    enemies_to_avoid_pos.append(i['pos'])
                

            if enemy_dis > 3 :
                next_move = astar(map_info, (bomberman[0], bomberman[1]), (near_enemy["pos"][0], near_enemy["pos"][1]), walls+enemies_to_avoid_pos)
                #print("astar path: {}".format(next_move))
                if next_move != None:
                    if len(next_move)>2:
                        key = go_to_next_block(next_move[1], bomberman)
            elif is_on_same_row_column(bomberman, near_enemy):
                key = "B" 

        else:
            #arranjar lista de inimigos para evitar, ou seja, todos menos o iniigo que vamos matar
            enemies_to_avoid_pos = []
            for i in enemies:
                if i['id']!=near_enemy['id']:
                    enemies_to_avoid_pos.append(i['pos'])
                
            if enemy_dis >= 2 :
                next_move = astar(map_info, (bomberman[0], bomberman[1]), (near_enemy["pos"][0], near_enemy["pos"][1]), walls+enemies_to_avoid_pos)
                #print("astar path: {}".format(next_move))
                if next_move != None:
                    if len(next_move)>2:
                        key = go_to_next_block(next_move[1], bomberman)
            elif is_on_same_row_column(bomberman, near_enemy):
                key = "B"   
    return key, obligatory_moves


def is_possible_to_kill_enemy(map_info, bomberman, enemy, walls, enemies_pos):
    temp_enemies_pos=enemies_pos
    if enemy['pos'] in temp_enemies_pos:
        temp_enemies_pos.remove(enemy['pos'])
    next_move = astar(map_info, (bomberman[0], bomberman[1]), (enemy["pos"][0], enemy["pos"][1]), walls+temp_enemies_pos)
    if next_move==[]:
        return False
    else:
        return True

def is_possible_to_destroy_wall(map_info, bomberman, wall_to_destroy, walls, enemies_pos):
    temp_walls=walls
    if wall_to_destroy in temp_walls:
        temp_walls.remove(wall_to_destroy)
    else:
        return False
    next_move = astar(map_info, (bomberman[0], bomberman[1]), (wall_to_destroy[0], wall_to_destroy[1]), temp_walls+enemies_pos)
    if next_move==[]:
        return False
    else:
        return True

def are_all_enemies_on_borders(game_properties, enemies_pos):
    for i in enemies_pos:
        if not ((i[1] == game_properties['size'][1]-2 or i[1] == 1) or (i[0] == game_properties['size'][0]-2 or i[0] == 1) ):
            return False
    return True

def are_walls_on_borders(game_properties, walls):
    for i in walls:
        if ((i[1] == game_properties['size'][1]-2 or i[1] == 1) or (i[0] == game_properties['size'][0]-2 or i[0] == 1) ):
            return i
    return None


def all_enemies_are_dumb(enemies):
    if enemies != []:
        for i in enemies:
            if not (i['name']=='Balloom' or i['name']=='Doll'):
                    return False
    
    return True

def find_closer_smart_enemy(map_info, bomberman, enemies, walls, enemies_pos):
    for i in enemies:
        if i['name']=='Oneal' or i['name']=='Minvo' or i['name']=='Ovapi':
            return i
    return None


def is_enemy_surrounded_except_by_one_block(mapa, near_enemy, walls):
    #except by one side
    #trying with different conditions in order to prevent deaths
    
    bx=near_enemy['pos'][0]
    by=near_enemy['pos'][1]
    counter=0

    blocks_adjs=[[bx-1, by], [bx+1, by], [bx, by-1], [bx, by+1]]
    for i in blocks_adjs:
        if mapa.is_stone(i) or i in walls:
            counter+=1
    if counter == 3:
        return True
    else:
        return False

#functions
#algoritmo de procura
def astar(maze, start, end, block_to_ignore=[]):
    #TO DO n sei se o limite ta adequado
    #var for safety---> not to enter in loop
    iter = 0
    
    
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    open_list = []
    closed_list = []

    open_list.append(start_node)

    while len(open_list) > 0:
        #save from possible loop
        #print("iter: ", iter) 
        iter+= 1
        if iter>len(maze)+len(maze[0]):
            return []

        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        open_list.pop(current_index)
    

        closed_list.append(current_node)

        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]: # Adjacent squares

            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # need to check after... not necessary changes for the first level
            if block_to_ignore==[]:
                if maze[node_position[0]][node_position[1]] == 1:
                    continue
            else:
                if maze[node_position[0]][node_position[1]] == 1 or [node_position[0],node_position[1]] in block_to_ignore:
                    continue
            new_node = Node(current_node, node_position)
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2) 
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)
#receber dados do jogo
def get_info(state):
    #print("----------------------------")
    #for key, value in state.items():
    #    print(key, value)
    return state['enemies'], state['walls'], state['bombs'], state['powerups'], state['bonus'], state['exit'], state['bomberman']
#calcular distancia entre dois pontos
def calculateDistance(c1, c2):
    c1x, c1y = c1
    c2x, c2y = c2

    return math.hypot(c1x - c2x, c1y - c2y)
# Função que calcula a parede que se encontra mais perto do bomberman
def find_closer_wall(bomberman, walls):
    distance = 2501
    
    for wall in walls:
        if calculateDistance(bomberman, wall) <= distance:
            distance = calculateDistance(bomberman, wall)
            final_wall = wall

    #print("FINAL_WALL: " , final_wall)
    return (final_wall, distance)
#retorna a key corresondente ao movimento para o bloco adjacente    
def go_to_next_block(new_block, bomberman):
    block_x, block_y = new_block
    b_x, b_y = bomberman
    if(block_y > b_y):
        #print("bloco em baixo")
        return 's'
    elif(block_y < b_y):
        #print("bloco em cima")
        return 'w'
    elif(block_x > b_x):
        #print("bloco à direita")
        return 'd'
    elif(block_x < b_x):
        #print("bloco a esquerda")
        return 'a'
    else:
        return ""

def find_near_enemy_killable(enemies, bomberman):
    #return closer enemy that's is not balloom
    if enemies == []:
        return None
    minimo = 999999   #numero exagerado de distancia
    near_enemy = None
    for enemy in enemies:
        dist = calculateDistance(bomberman, enemy["pos"])
        if(enemy['name']!='Balloom' or enemy['name']!='Doll'):
            if(dist < minimo):
                minimo=dist
                near_enemy=enemy
    return near_enemy, minimo
#retorna o inimigo mais perto
def find_near_enemy(enemies, bomberman):
    if enemies == []:
        return None
    minimo = 999999   #numero exagerado de distancia
    near_enemy = None
    for enemy in enemies:
        #print(enemy['pos'][0])
        #print(bomberman)
        dist = calculateDistance(bomberman, enemy["pos"])
        #print('dist:')
        #print(dist)
        if(dist < minimo):
            minimo=dist
            near_enemy=enemy
    return near_enemy, minimo
#retorna True de existir bomba
def exists_bomb(bombs):
    if bombs == []:
        return False
    else:
        return True
#retorna True se tiver obtido powerup "DETONATOR"
def has_detonator(powerups_obtained):
    for i in powerups_obtained:
        if i == "Detonator":
            return True
    return False
#verifica se exite uma wall na posicao indicada
def is_destructible_Wall(pos, walls):
    if walls != []:
        for i in walls:
            if i == pos:
                #print("[Destructible Wall]: ", pos)
                return True
        
    return False
#verifica se existe um inimigo na posicoao indicada
def is_enemy(pos, enemies):
    if enemies != []:
        for i in enemies:
            #print(i['pos']  )
            if i['pos'] == pos:
                return True
    
    return False
#verifica se existe uma wall ou inimigo na posicao indicada
def is_enemy_or_destructible_Wall(pos, walls, enemies):
    if is_enemy(pos, enemies) or is_destructible_Wall(pos, walls):
        return True
    return False
#infomcacao relativa à explosaoa da bombas
def bomb_info(bombs, mapa):
    blow_directions = []
    move_count = 1
    
    for bomb in bombs:
        coord, timeout, radius = bomb
        x, y = coord

        # determina as direções para as quais a bomba vai explodir
        if not mapa.is_stone([x + 1, y]):
            blow_directions.append("d")
        if not mapa.is_stone([x - 1, y]):
            blow_directions.append("a")
        if not mapa.is_stone([x, y + 1]):
            blow_directions.append("s")
        if not mapa.is_stone([x, y - 1]):
            blow_directions.append("w")

        # determina se a bomba está nas diagonais das paredes indestrutiveis || terá que se mover num L
        if mapa.is_stone([x + 1, y + 1]) and mapa.is_stone([x + 1, y - 1]) and mapa.is_stone([x - 1, y + 1]) and mapa.is_stone([x - 1, y - 1]):
            move_count = 2

    information = (blow_directions, move_count, bombs[0][2])
    #print("move count: ", move_count)
    #print("blow_directions", blow_directions)
    
    return information, radius
#goal is to return a list of movements that lead to a safe place instead of safe places itselfs
def bomb_run_v2(information, bomberman, mapa, near_enemy, walls, movements_array_to_safety, enemies, map_info, security_distance):
    blow_directions, move_count, radius = information
    radius = radius + 1
    #print("[radius]:", radius)
    #print("in bomb_run, enemies: ", enemies)

    b_x, b_y = bomberman
                                #avaliar todos os bloco ate ao bloco safe no caso forward 
    for b in blow_directions: 
        if move_count == 1:
            if b == "w":
                diag_left = [b_x - 1, b_y - 1]
                diag_right = [b_x + 1, b_y - 1]
                forward = [b_x, b_y - radius]
                intermediate = [b_x, b_y -1]

                if not (mapa.is_stone(intermediate)) and not is_enemy_or_destructible_Wall(intermediate, walls, enemies):
                    if not (mapa.is_stone(diag_left)) and not is_enemy_or_destructible_Wall(diag_left, walls, enemies):
                        if enemies != []:
                            near_enemy, enemy_dis = find_near_enemy(enemies, diag_left)
                            if calculateDistance(diag_left, near_enemy["pos"]) > security_distance:
                                movements_array_to_safety.append(intermediate)
                                movements_array_to_safety.append(diag_left)
                                return movements_array_to_safety
                        else:
                            movements_array_to_safety.append(intermediate)
                            movements_array_to_safety.append(diag_left)
                            return movements_array_to_safety
                    elif not (mapa.is_stone(diag_right)) and not is_enemy_or_destructible_Wall(diag_right, walls, enemies):
                        if enemies != []:
                            near_enemy, enemy_dis = find_near_enemy(enemies, diag_right)
                            if calculateDistance(diag_right, near_enemy["pos"]) > security_distance:
                                movements_array_to_safety.append(intermediate)
                                movements_array_to_safety.append(diag_right)
                                return movements_array_to_safety
                        else:
                            movements_array_to_safety.append(intermediate)
                            movements_array_to_safety.append(diag_right)
                            return movements_array_to_safety
                    elif arrays_of_blocks_are_safe_from_explosion(mapa, map_info, [b_x,b_y], [forward[0],forward[1]], walls, enemies):
                        if enemies != []:
                            near_enemy, enemy_dis = find_near_enemy(enemies, forward)
                            if calculateDistance(forward, near_enemy["pos"]) > security_distance:
                                array = astar(map_info, (b_x,b_y), (forward[0],forward[1]))                          
                                for move in array:
                                    movements_array_to_safety.append(move)
                                return movements_array_to_safety
                        else:
                            array = astar(map_info, (b_x,b_y), (forward[0],forward[1]))                          
                            for move in array:
                                movements_array_to_safety.append(move)
                            return movements_array_to_safety
        
            if b == "a":
                diag_up = [b_x - 1, b_y - 1]
                diag_down = [b_x - 1, b_y + 1]
                forward = [b_x - radius, b_y]
                intermediate = [b_x - 1, b_y]

                if not (mapa.is_stone(intermediate)) and not is_enemy_or_destructible_Wall(intermediate, walls, enemies):
                    if not (mapa.is_stone(diag_up)) and not is_enemy_or_destructible_Wall(diag_up, walls, enemies):
                        if enemies != []:
                            near_enemy, enemy_dis = find_near_enemy(enemies, diag_up)
                            if calculateDistance(diag_up, near_enemy["pos"]) > security_distance:
                                movements_array_to_safety.append(intermediate)
                                movements_array_to_safety.append(diag_up)
                                return movements_array_to_safety
                        else:
                            movements_array_to_safety.append(intermediate)
                            movements_array_to_safety.append(diag_up)
                            return movements_array_to_safety
                    
                    
                    elif not (mapa.is_stone(diag_down)) and not is_enemy_or_destructible_Wall(diag_down, walls, enemies):
                        if enemies != []:
                            near_enemy, enemy_dis = find_near_enemy(enemies, diag_down)
                            if calculateDistance(diag_down, near_enemy["pos"]) > security_distance:
                                movements_array_to_safety.append(intermediate)
                                movements_array_to_safety.append(diag_down)
                                return movements_array_to_safety
                        else:
                            movements_array_to_safety.append(intermediate)
                            movements_array_to_safety.append(diag_down)
                            return movements_array_to_safety


                    elif arrays_of_blocks_are_safe_from_explosion(mapa, map_info, [b_x,b_y], [forward[0],forward[1]], walls, enemies):
                        if enemies != []:
                            near_enemy, enemy_dis = find_near_enemy(enemies, forward)
                            if calculateDistance(forward, near_enemy["pos"]) > security_distance:
                                array = astar(map_info, (b_x,b_y), (forward[0],forward[1]))
                                for move in array:
                                    movements_array_to_safety.append(move)
                                return movements_array_to_safety
                        else:
                            array = astar(map_info, (b_x,b_y), (forward[0],forward[1]))
                            for move in array:
                                movements_array_to_safety.append(move)
                            return movements_array_to_safety

            if b == "d":
                diag_up = [b_x + 1, b_y - 1]
                diag_down = [b_x + 1, b_y + 1]
                intermediate = [b_x + 1, b_y]
                forward = [b_x + radius, b_y]


                if not (mapa.is_stone(intermediate)) and not is_enemy_or_destructible_Wall(intermediate, walls, enemies):
                    if not (mapa.is_stone(diag_up)) and not is_enemy_or_destructible_Wall(diag_up, walls, enemies):
                        if enemies != []:
                            near_enemy, enemy_dis = find_near_enemy(enemies, diag_up)   
                            if calculateDistance(diag_up, near_enemy["pos"]) > security_distance:
                                movements_array_to_safety.append(intermediate)
                                movements_array_to_safety.append(diag_up)
                                return movements_array_to_safety
                        else:
                            movements_array_to_safety.append(intermediate)
                            movements_array_to_safety.append(diag_up)
                            return movements_array_to_safety                    
                    
                    elif not (mapa.is_stone(diag_down)) and not is_enemy_or_destructible_Wall(diag_down, walls, enemies):
                        if enemies != []:
                            near_enemy, enemy_dis = find_near_enemy(enemies, diag_down)
                            if calculateDistance(diag_down, near_enemy["pos"]) > security_distance:
                                movements_array_to_safety.append(intermediate)
                                movements_array_to_safety.append(diag_down)
                                return movements_array_to_safety
                        else:
                            movements_array_to_safety.append(intermediate)
                            movements_array_to_safety.append(diag_down)
                            return movements_array_to_safety

                    elif arrays_of_blocks_are_safe_from_explosion(mapa, map_info, [b_x,b_y], [forward[0],forward[1]], walls, enemies):
                        if enemies != []:
                            near_enemy, enemy_dis = find_near_enemy(enemies, forward)
                            if calculateDistance(forward, near_enemy["pos"]) > security_distance:
                                array = astar(map_info, (b_x,b_y), (forward[0],forward[1]))
                                for move in array:
                                    movements_array_to_safety.append(move)
                                return movements_array_to_safety
                        else:
                            array = astar(map_info, (b_x,b_y), (forward[0],forward[1]))
                            for move in array:
                                movements_array_to_safety.append(move)
                            return movements_array_to_safety

            if b == "s":
                diag_left = [b_x - 1, b_y + 1]
                diag_right = [b_x + 1, b_y + 1]
                forward = [b_x, b_y + radius]
                intermediate = [b_x, b_y + 1]

                if not (mapa.is_stone(intermediate)) and not is_enemy_or_destructible_Wall(intermediate, walls, enemies):
                    if not (mapa.is_stone(diag_left)) and not is_enemy_or_destructible_Wall(diag_left, walls, enemies):
                        if enemies != []:
                            near_enemy, enemy_dis = find_near_enemy(enemies, diag_left)
                            if calculateDistance(diag_left, near_enemy["pos"]) > security_distance:
                                movements_array_to_safety.append(intermediate)
                                movements_array_to_safety.append(diag_left)
                                return movements_array_to_safety  
                        else:
                            movements_array_to_safety.append(intermediate)
                            movements_array_to_safety.append(diag_left)
                            return movements_array_to_safety  

                    elif not (mapa.is_stone(diag_right)) and not is_enemy_or_destructible_Wall(diag_right, walls, enemies):
                        if enemies != []:
                            near_enemy, enemy_dis = find_near_enemy(enemies, diag_right)
                            if calculateDistance(diag_right, near_enemy["pos"]) > security_distance:
                                movements_array_to_safety.append(intermediate)
                                movements_array_to_safety.append(diag_right)
                                return movements_array_to_safety
                        else:
                            movements_array_to_safety.append(intermediate)
                            movements_array_to_safety.append(diag_right)
                            return movements_array_to_safety
                    elif arrays_of_blocks_are_safe_from_explosion(mapa, map_info, [b_x,b_y], [forward[0],forward[1]], walls, enemies):
                        if enemies != []:
                            near_enemy, enemy_dis = find_near_enemy(enemies, forward)
                            if calculateDistance(forward, near_enemy["pos"]) > security_distance:
                                array = astar(map_info, (b_x,b_y), (forward[0],forward[1]))
                                for move in array:
                                    movements_array_to_safety.append(move)
                                return movements_array_to_safety
                        else:
                            array = astar(map_info, (b_x,b_y), (forward[0],forward[1]))
                            for move in array:
                                movements_array_to_safety.append(move)
                            return movements_array_to_safety
        if move_count == 2:
            if b == "w":
                diag_left = [b_x - 1, b_y - 2]
                diag_right = [b_x + 1, b_y - 2]
                intermediate = [b_x, b_y - 2]
                intermediate2 = [b_x, b_y - 1]
                forward = [b_x, b_y - radius]
                if not mapa.is_stone(intermediate) and not mapa.is_stone(intermediate2) and not is_enemy_or_destructible_Wall(intermediate, walls, enemies) and not is_enemy_or_destructible_Wall(intermediate2, walls, enemies) :
                    if not mapa.is_stone(diag_left) and not is_enemy_or_destructible_Wall(diag_left, walls, enemies):
                        if enemies != []:
                            near_enemy, enemy_dis = find_near_enemy(enemies, diag_left)
                            if calculateDistance(diag_left, near_enemy["pos"]) > security_distance:    
                                movements_array_to_safety.append(intermediate2)
                                movements_array_to_safety.append(intermediate)
                                movements_array_to_safety.append(diag_left)
                                return movements_array_to_safety
                        else:
                            movements_array_to_safety.append(intermediate2)
                            movements_array_to_safety.append(intermediate)
                            movements_array_to_safety.append(diag_left)
                            return movements_array_to_safety
                    
                    elif not mapa.is_stone(diag_right) and not is_enemy_or_destructible_Wall(diag_right, walls, enemies):
                        if enemies != []:
                            near_enemy, enemy_dis = find_near_enemy(enemies, diag_right)
                            if calculateDistance(diag_right, near_enemy["pos"]) > security_distance:
                                movements_array_to_safety.append(intermediate2)
                                movements_array_to_safety.append(intermediate)
                                movements_array_to_safety.append(diag_right)
                                return movements_array_to_safety
                        else:
                            movements_array_to_safety.append(intermediate2)
                            movements_array_to_safety.append(intermediate)
                            movements_array_to_safety.append(diag_right)
                            return movements_array_to_safety
                    elif arrays_of_blocks_are_safe_from_explosion(mapa, map_info, [b_x,b_y], [forward[0],forward[1]], walls, enemies):
                        if enemies != []:
                            near_enemy, enemy_dis = find_near_enemy(enemies, forward)
                            if calculateDistance(forward, near_enemy["pos"]) > security_distance:
                                array = astar(map_info, (b_x,b_y), (forward[0],forward[1]))                          
                                for move in array:
                                    movements_array_to_safety.append(move)
                                return movements_array_to_safety
                        else:
                            array = astar(map_info, (b_x,b_y), (forward[0],forward[1]))                          
                            for move in array:
                                movements_array_to_safety.append(move)
                            return movements_array_to_safety
                            
                   
            if b == "a":
                diag_up = [b_x - 2, b_y - 1]
                diag_down = [b_x - 2, b_y + 1]
                intermediate = [b_x - 2, b_y]
                intermediate2 = [b_x - 1, b_y]
                forward = [b_x - radius, b_y]
                if not mapa.is_stone(intermediate) and not mapa.is_stone(intermediate2) and not is_enemy_or_destructible_Wall(intermediate, walls, enemies) and not is_enemy_or_destructible_Wall(intermediate2, walls, enemies):
                    if not mapa.is_stone(diag_up) and not is_enemy_or_destructible_Wall(diag_up, walls, enemies):  
                        if enemies != []:
                            near_enemy, enemy_dis = find_near_enemy(enemies, diag_up)
                            if calculateDistance(diag_up, near_enemy["pos"]) > security_distance:
                                movements_array_to_safety.append(intermediate2)
                                movements_array_to_safety.append(intermediate)
                                movements_array_to_safety.append(diag_up)
                                return movements_array_to_safety
                        else:
                            movements_array_to_safety.append(intermediate2)
                            movements_array_to_safety.append(intermediate)
                            movements_array_to_safety.append(diag_up)
                            return movements_array_to_safety
                    
                    elif not mapa.is_stone(diag_down) and not is_enemy_or_destructible_Wall(diag_down, walls, enemies): 
                        if enemies != []:
                            near_enemy, enemy_dis = find_near_enemy(enemies, diag_down)
                            if calculateDistance(diag_down, near_enemy["pos"]) > security_distance:
                                movements_array_to_safety.append(intermediate2)
                                movements_array_to_safety.append(intermediate)
                                movements_array_to_safety.append(diag_down)
                                return movements_array_to_safety
                        else:
                            movements_array_to_safety.append(intermediate2)
                            movements_array_to_safety.append(intermediate)
                            movements_array_to_safety.append(diag_down)
                            return movements_array_to_safety
                    elif arrays_of_blocks_are_safe_from_explosion(mapa, map_info, [b_x,b_y], [forward[0],forward[1]], walls, enemies):
                        if enemies != []:
                            near_enemy, enemy_dis = find_near_enemy(enemies, forward)
                            if calculateDistance(forward, near_enemy["pos"]) > security_distance:
                                array = astar(map_info, (b_x,b_y), (forward[0],forward[1]))                          
                                for move in array:
                                    movements_array_to_safety.append(move)
                                return movements_array_to_safety
                        else:
                            array = astar(map_info, (b_x,b_y), (forward[0],forward[1]))                          
                            for move in array:
                                movements_array_to_safety.append(move)
                            return movements_array_to_safety
            if b == "d":
                diag_up = [b_x + 2, b_y - 1]
                diag_down = [b_x + 2, b_y + 1]
                intermediate = [b_x + 2, b_y]
                intermediate2 = [b_x + 1, b_y]
                forward = [b_x + radius, b_y]
                if not (mapa.is_stone(intermediate) and mapa.is_stone(intermediate2)) and not is_enemy_or_destructible_Wall(intermediate, walls, enemies) and not is_enemy_or_destructible_Wall(intermediate2, walls, enemies):
                    if not mapa.is_stone(diag_up) and not is_enemy_or_destructible_Wall(diag_up, walls, enemies):
                        if enemies != []:
                            near_enemy, enemy_dis = find_near_enemy(enemies, diag_up)
                            if calculateDistance(diag_up, near_enemy["pos"]) > security_distance:
                                movements_array_to_safety.append(intermediate2)
                                movements_array_to_safety.append(intermediate)
                                movements_array_to_safety.append(diag_up)
                                return movements_array_to_safety
                        else:
                            movements_array_to_safety.append(intermediate2)
                            movements_array_to_safety.append(intermediate)
                            movements_array_to_safety.append(diag_up)
                            return movements_array_to_safety
                   
                    elif not mapa.is_stone(diag_down) and not is_enemy_or_destructible_Wall(diag_down, walls, enemies):
                        if enemies != []:
                            near_enemy, enemy_dis = find_near_enemy(enemies, diag_down)
                            if calculateDistance(diag_down, near_enemy["pos"]) > security_distance:
                                movements_array_to_safety.append(intermediate2)
                                movements_array_to_safety.append(intermediate)
                                movements_array_to_safety.append(diag_down)
                                return movements_array_to_safety
                        else:
                            movements_array_to_safety.append(intermediate2)
                            movements_array_to_safety.append(intermediate)
                            movements_array_to_safety.append(diag_down)
                            return movements_array_to_safety

                    elif arrays_of_blocks_are_safe_from_explosion(mapa, map_info, [b_x,b_y], [forward[0],forward[1]], walls, enemies):
                        if enemies != []:
                            near_enemy, enemy_dis = find_near_enemy(enemies, forward)
                            if calculateDistance(forward, near_enemy["pos"]) > security_distance:
                                array = astar(map_info, (b_x,b_y), (forward[0],forward[1]))                          
                                for move in array:
                                    movements_array_to_safety.append(move)
                                return movements_array_to_safety
                        else:
                            array = astar(map_info, (b_x,b_y), (forward[0],forward[1]))                          
                            for move in array:
                                movements_array_to_safety.append(move)
                            return movements_array_to_safety
            if b == "s":
                diag_left = [b_x - 1, b_y + 2]
                diag_right = [b_x + 1, b_y + 2]
                intermediate = [b_x, b_y + 2]
                intermediate2 = [b_x, b_y + 1]
                forward = [b_x, b_y + radius]
                if not mapa.is_stone(intermediate) and not mapa.is_stone(intermediate2) and not is_enemy_or_destructible_Wall(intermediate, walls, enemies) and not is_enemy_or_destructible_Wall(intermediate2, walls, enemies):
                    if not mapa.is_stone(diag_left) and not is_enemy_or_destructible_Wall(diag_left, walls, enemies):
                        if enemies != []:
                            near_enemy, enemy_dis = find_near_enemy(enemies, diag_left)
                            if calculateDistance(diag_left, near_enemy["pos"]) > security_distance:
                                movements_array_to_safety.append(intermediate2)
                                movements_array_to_safety.append(intermediate)
                                movements_array_to_safety.append(diag_left)
                                return movements_array_to_safety
                        else:
                            movements_array_to_safety.append(intermediate2)
                            movements_array_to_safety.append(intermediate)
                            movements_array_to_safety.append(diag_left)
                            return movements_array_to_safety
                    elif not mapa.is_stone(diag_right) and not is_enemy_or_destructible_Wall(diag_right, walls, enemies):
                        if enemies != []:
                            near_enemy, enemy_dis = find_near_enemy(enemies, diag_right)
                            if calculateDistance(diag_right, near_enemy["pos"]) > security_distance:
                                movements_array_to_safety.append(intermediate2)
                                movements_array_to_safety.append(intermediate)
                                movements_array_to_safety.append(diag_right)
                                return movements_array_to_safety
                        else:
                            movements_array_to_safety.append(intermediate2)
                            movements_array_to_safety.append(intermediate)
                            movements_array_to_safety.append(diag_right)
                            return movements_array_to_safety
                    elif arrays_of_blocks_are_safe_from_explosion(mapa, map_info, [b_x,b_y], [forward[0],forward[1]], walls, enemies):
                        if enemies != []:
                            near_enemy, enemy_dis = find_near_enemy(enemies, forward)
                            if calculateDistance(forward, near_enemy["pos"]) > security_distance:
                                array = astar(map_info, (b_x,b_y), (forward[0],forward[1]))                          
                                for move in array:
                                    movements_array_to_safety.append(move)
                                return movements_array_to_safety
                        else:
                            array = astar(map_info, (b_x,b_y), (forward[0],forward[1]))                          
                            for move in array:
                                movements_array_to_safety.append(move)
                            return movements_array_to_safety

                    


    #in case he's trapped
    movements_array_to_safety = []
    


    return movements_array_to_safety
#receber lista de inimigoss
def get_enemies_list(state):
    enemies_pos=[]
    for i in state['enemies']:
        enemies_pos += [i['pos']] 
    return enemies_pos
#verificar se uma linha reta esta safa de explosoes e n  contem inimigos nem paredes
def arrays_of_blocks_are_safe_from_explosion(mapa, map_info, origin, final, walls, enemies):
    path=[]
    example=origin
    path.append(example)
    if final[0]>origin[0]:
        while example != final:
            example = [example[0]+1, example[1]]
            path.append(example)
    elif final[0]<origin[0]:
        while example != final:
            example = [example[0]-1, example[1]]
            path.append(example)
    elif final[1]>origin[1]:
        while example != final:
            example = [example[0], example[1]+1]
            path.append(example)
    elif final[1]<origin[1]:
        while example != final:
            example = [example[0], example[1]-1]
            path.append(example)
    
    #print("PATH: (linha828)")
    for i in path:   
        #print("-> ", i)
        if mapa.is_stone(i) or is_enemy_or_destructible_Wall(i, walls, enemies):
            #print("nao e valido() parede ou inimigo: ", i)
            return False

    return True

def exists_Dumb_Enemies(enemies):
    for i in enemies:
        if i['name']=='Balloom' or i['name']=='Doll' :
            return True
    return False

def is_enemy_on_corner(game_properties, closest_enemy, bomberman):
    #return a position rigth next to the enemy or None if he isn't on the corner
    if closest_enemy['pos']==[1,1] :
        return [1, 2] if calculateDistance(bomberman, [1,2]) <= calculateDistance(bomberman, [2,1]) else [2,1]
    if closest_enemy['pos']==[1, game_properties['size'][1]-2] :
        return [2, game_properties['size'][1]-2] if calculateDistance(bomberman, [2, game_properties['size'][1]-2]) <= calculateDistance(bomberman, [1, game_properties['size'][1]-3]) else [1, game_properties['size'][1]-3]
    if closest_enemy['pos']==[game_properties['size'][0]-2, 1] :
        return [game_properties['size'][0]-2, 2] if calculateDistance(bomberman, [game_properties['size'][0]-2, 2]) <= calculateDistance(bomberman, [game_properties['size'][0]-3, 1]) else [game_properties['size'][0]-3, 1]
    if closest_enemy['pos']==[game_properties['size'][0]-2, game_properties['size'][1]-2]:
        return [game_properties['size'][0]-3, game_properties['size'][1]-2] if calculateDistance(bomberman, [game_properties['size'][0]-3, game_properties['size'][1]-2]) <= calculateDistance(bomberman, [game_properties['size'][0]-2, game_properties['size'][1]-3]) else  [game_properties['size'][0]-2, game_properties['size'][1]-3]
    return None

def is_on_same_row_column(bomberman, enemy):
    if( bomberman[0]== enemy['pos'][0] or bomberman[1] == enemy['pos'][1]) and calculateDistance(bomberman, enemy['pos']) < 2 :
        return True
    return False
# DO NOT CHANGE THE LINES BELLOW
# You can change the default values using the command line, example:
# $ NAME='bombastico' python3 client.py
loop = asyncio.get_event_loop()
SERVER = os.environ.get("SERVER", "localhost")
PORT = os.environ.get("PORT", "8000")
NAME = os.environ.get("NAME", getpass.getuser())
loop.run_until_complete(agent_loop(f"{SERVER}:{PORT}", NAME))
