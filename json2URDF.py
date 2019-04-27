import json
import numpy as np

#this funciton is used to obtain the link part URDF code
def add_link(label, mesh, origin):
    temp_URDF = []
    temp_URDF.append(f'\t<link name="{label}{mesh}">')
    temp_URDF.append('\t\t<visual>')
    temp_URDF.append(f'\t\t\t<origin xyz="{-origin[0]} {-origin[1]} {-origin[2]}" rpy="0 0 0"/>')
    #temp_URDF.append(f'\t\t\t<origin xyz="0 0 0" rpy="0 0 0"/>')
    temp_URDF.append('\t\t\t<geometry>')
    temp_URDF.append(f'\t\t\t\t<mesh filename="./raw_data/part_dae/{mesh}.dae" />')
    temp_URDF.append('\t\t\t</geometry>')
    temp_URDF.append('\t\t</visual>')  
    temp_URDF.append('\t</link>')    
    temp_URDF.append('\n')    
    return temp_URDF

def add_joint(joint_type, parent_link, children_link, origin, axis=[], rangeMin=0, rangeMax=0):
    temp_URDF = []
    #fixed joint
    if joint_type == 0:
        temp_URDF.append(f'\t<joint name="{parent_link}_{children_link}" type="fixed">')
        temp_URDF.append(f'\t\t<parent link="{parent_link}"/>')
        temp_URDF.append(f'\t\t<child link="{children_link}"/>')
        temp_URDF.append(f'\t\t<origin xyz="{origin[0]} {origin[1]} {origin[2]}" rpy="0 0 0"/>')
        temp_URDF.append('\t</joint>')
        temp_URDF.append('\n')  
        return temp_URDF
    #Central Rotation
    elif joint_type == 1:
        temp_URDF.append(f'\t<joint name="{parent_link}_{children_link}_rotation" type="revolute">')  
        temp_URDF.append(f'\t\t<parent link="{parent_link}"/>')  
        temp_URDF.append(f'\t\t<child link="{children_link}"/>')  
        temp_URDF.append(f'\t\t<origin xyz="{origin[0]} {origin[1]} {origin[2]}" rpy="0 0 0"/>')  
        temp_URDF.append(f'\t\t<limit lower="{rangeMin}" upper="{rangeMax}" effort="10" velocity="3"/>')  
        temp_URDF.append(f'\t\t<axis xyz="{axis[0]} {axis[1]} {axis[2]}"/>')  
        temp_URDF.append('\t</joint>')  
        temp_URDF.append('\n')  
        return temp_URDF
    #Translation
    elif joint_type == 2:
        temp_URDF.append(f'\t<joint name="{parent_link}_{children_link}_prismatic" type="prismatic">')  
        temp_URDF.append(f'\t\t<parent link="{parent_link}"/>')  
        temp_URDF.append(f'\t\t<child link="{children_link}"/>')  
        temp_URDF.append(f'\t\t<origin xyz="{origin[0]} {origin[1]} {origin[2]}" rpy="0 0 0"/>')  
        temp_URDF.append(f'\t\t<limit lower="{rangeMin}" upper="{rangeMax}" effort="10" velocity="3"/>')  
        temp_URDF.append(f'\t\t<axis xyz="{axis[0]} {axis[1]} {axis[2]}"/>')  
        temp_URDF.append('\t</joint>')  
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
    
    #the relationship between the id of the pid and meshes and the pid with the label
    get_mesh = np.zeros(len(connectivity_graph), dtype=int)
    get_label = {}
    nodes = ajson_data['nodes']
    for i in nodes:
        if i['name'].isdigit():
            get_mesh[int(i['name'])] = i['id']
            get_label[int(i['name'])] = i['label']

    #get the information of some parts which should be the children link,0:x, 1:y, 2:z
    part_num = len(connectivity_graph)
    joint_type = np.zeros((part_num, part_num), dtype=int) #0:fixed, 1:"Central Rotation", 2:"Translation"
    origin = np.zeros((part_num, 3))
    axis = np.zeros((part_num, 3))
    rangeMin = np.zeros(part_num)
    rangeMax = np.zeros(part_num)
    parent = np.zeros(part_num, dtype=int)

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

        parent = int(i['base'][0])

        #joint_type[i, j]: i is children link, j is parent link; 0:fixed, 1:rotation, 2:translation, -1: no joint
        if i['type'] == 'Central Rotation':
            joint_type[int(i['pid']), parent] = 1
            joint_type[parent, int(i['pid'])] = -1
        elif i['type'] == 'Translation':
            joint_type[int(i['pid']), parent] = 2
            joint_type[parent, int(i['pid'])] = -1


    #used to write the URDF
    URDF = []
    URDF.append(f'<robot name="{object_id}">')
    URDF.append('\n')

    #used to add all the links
    for i in nodes:
        if i['name'].isdigit():
            temp_URDF = add_link(i['label'], int(i['id']), origin[int(i['name'])])
            URDF.extend(temp_URDF)

    #used to add all joints(traverse the connectivity graph)
    for i in range(len(connectivity_graph)):
        for j in connectivity_graph[i]:
            if joint_type[i, j] == -1:
                continue
            elif joint_type[i, j] == 0:
                temp_URDF = add_joint(0, get_label[j]+str(get_mesh[j]), get_label[i]+str(get_mesh[i]), origin[i]-origin[j])
                URDF.extend(temp_URDF)
                joint_type[i, j] = -1
                joint_type[j, i] = -1
            elif joint_type[i, j] == 1:
                temp_URDF = add_joint(1, get_label[j]+str(get_mesh[j]), get_label[i]+str(get_mesh[i]), origin[i]-origin[j], axis[i], rangeMin[i], rangeMax[i])
                URDF.extend(temp_URDF)
                joint_type[i, j] = -1
                joint_type[j, i] = -1
            elif joint_type[i, j] == 2:
                temp_URDF = add_joint(2, get_label[j]+str(get_mesh[j]), get_label[i]+str(get_mesh[i]), origin[i]-origin[j], axis[i], rangeMin[i], rangeMax[i])
                URDF.extend(temp_URDF)
                joint_type[i, j] = -1
                joint_type[j, i] = -1

    URDF.append(f'</robot>')

    file = open(f'{object_id}.URDF', 'w')
    file.write('\n'.join(URDF))
    file.close()

