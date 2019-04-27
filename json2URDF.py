import json
import numpy as np

#this funciton is used to obtain the link part URDF code
def add_link(label, mesh, origin):
    temp_URDF = []
    temp_URDF.append(f'\t<link name="{mesh}_{label}">')
    temp_URDF.append('\t\t<visual>')
    #temp_URDF.append(f'\t\t\t<origin xyz="{-origin[0]} {-origin[1]} {-origin[2]}" rpy="0 0 0"/>')
    temp_URDF.append(f'\t\t\t<origin xyz="0 0 0" rpy="0 0 0"/>')
    temp_URDF.append('\t\t\t<geometry>')
    temp_URDF.append(f'\t\t\t\t<mesh filename="./raw_data/part_dae/{mesh}.dae" />')
    temp_URDF.append('\t\t\t</geometry>')
    temp_URDF.append('\t\t</visual>')  
    temp_URDF.append('\t</link>')    
    temp_URDF.append('\n')    
    return temp_URDF


if __name__ == '__main__':
    object_id = 106

    #ajson file is used to get the relationship between the link and the meshes
    file = open(f'./raw_data/{object_id}.ajson')
    ajson_data = json.load(file)
    file.close()

    #json file is used to get the articulations relationship
    file = open(f'./raw_data/p5d.{object_id}.articulations.json')
    json_data = json.load(file)
    file.close()

    #artpre.json file is used to get the connectivity graph
    file = open(f'./raw_data/{object_id}.artpre.json')
    artpre_data = json.load(file)
    file.close()

    #store the connectivity graph for the fixed articulations
    connectivity_graph = artpre_data['connectivityGraph']
    
    #the relationship between the id of the pid and meshes
    get_mesh = np.zeros(len(connectivity_graph), dtype=int)
    nodes = ajson_data['nodes']
    for i in nodes:
        if i['name'].isdigit():
            get_mesh[int(i['name'])] = i['id']

    #get the information of some parts which should be the children link,0:x, 1:y, 2:z
    origin = np.zeros((len(connectivity_graph), 3))
    axis = np.zeros((len(connectivity_graph), 3))
    rangeMin = np.zeros(len(connectivity_graph))
    rangeMax = np.zeros(len(connectivity_graph))

    part_info = json_data['data']['articulations']
    for i in part_info:
        axis[int(i['pid'])][0] = float(i['axis']['x'])
        axis[int(i['pid'])][1] = float(i['axis']['y'])
        axis[int(i['pid'])][2] = float(i['axis']['z'])

        origin[int(i['pid'])][0] = float(i['origin']['x'])
        origin[int(i['pid'])][1] = float(i['origin']['y'])
        origin[int(i['pid'])][2] = float(i['origin']['z'])

        rangeMin[int(i['pid'])] = float(i['rangeMin'])
        rangeMax[int(i['pid'])] = float(i['rangeMax'])


    #used to write the URDF
    URDF = []
    URDF.append(f'<robot name="{object_id}">')
    URDF.append('\n')

    #used to add all the links
    for i in nodes:
        if i['name'].isdigit():
            temp_URDF = add_link(i['label'], int(i['id']), origin[int(i['name'])])
            URDF.extend(temp_URDF)
    
    URDF.append(f'</robot>')

    file = open(f'{object_id}.URDF', 'w')
    file.write('\n'.join(URDF))
    file.close()

