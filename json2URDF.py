import json
import numpy as np
import argparse
import glob
import os

#this funciton is used to obtain the link part URDF code
def add_link(label, mesh, origin, dir):
    temp_URDF = []
    temp_URDF.append(f'\t<link name="{mesh}:{label}">')

    temp_URDF.append('\t\t<inertial>')
    temp_URDF.append('\t\t\t<origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>')
    temp_URDF.append('\t\t\t<mass value="1.0"/>')
    temp_URDF.append('\t\t\t<inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>')
    temp_URDF.append('\t\t</inertial>')  

    temp_URDF.append('\t\t<visual>')
    temp_URDF.append(f'\t\t\t<origin xyz="{-origin[0]} {-origin[1]} {-origin[2]}" rpy="0 0 0"/>')
    temp_URDF.append('\t\t\t<geometry>')
    temp_URDF.append(f'\t\t\t\t<mesh filename="../{dir}/{object_id}/part_meshes/{mesh}.dae" />')
    temp_URDF.append('\t\t\t</geometry>')
    temp_URDF.append('\t\t</visual>')  
    temp_URDF.append('\t</link>')    
    temp_URDF.append('\n')    
    return temp_URDF

def add_joint(joint_type, parent_link, children_link, origin, axis=[], rangeMin=0, rangeMax=0):
    temp_URDF = []
    #fixed joint
    if joint_type == 0:
        temp_URDF.append(f'\t<joint name="{parent_link}::{children_link}" type="fixed">')
        temp_URDF.append(f'\t\t<parent link="{parent_link}"/>')
        temp_URDF.append(f'\t\t<child link="{children_link}"/>')
        temp_URDF.append(f'\t\t<origin xyz="{origin[0]} {origin[1]} {origin[2]}" rpy="0 0 0"/>')
        temp_URDF.append('\t</joint>')
        temp_URDF.append('\n')  
        return temp_URDF
    #Central Rotation and Hinged Rotation
    elif joint_type == 1:
        temp_URDF.append(f'\t<joint name="{parent_link}::{children_link}_rotation" type="revolute">')  
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
        temp_URDF.append(f'\t<joint name="{parent_link}::{children_link}_translation" type="prismatic">')  
        temp_URDF.append(f'\t\t<parent link="{parent_link}"/>')  
        temp_URDF.append(f'\t\t<child link="{children_link}"/>')  
        temp_URDF.append(f'\t\t<origin xyz="{origin[0]} {origin[1]} {origin[2]}" rpy="0 0 0"/>')  
        temp_URDF.append(f'\t\t<limit lower="{rangeMin}" upper="{rangeMax}" effort="10" velocity="3"/>')  
        temp_URDF.append(f'\t\t<axis xyz="{axis[0]} {axis[1]} {axis[2]}"/>')  
        temp_URDF.append('\t</joint>')  
        temp_URDF.append('\n')  
        return temp_URDF



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    #parser.add_argument('-i','--id', type=str, help='the object which is used to trandform into URDF, such as 106', required=False, default='397')
    parser.add_argument('-d','--data_dir', type=str, help='the dir which contains all the data, suach as ./raw_data', required=False, default='raw_data')
    args = parser.parse_args()

    dir = args.data_dir

    model_dirs = glob.glob(dir + '/*')
    for model_dir in model_dirs:

        #convert all meshes into dae form
        os.system("cd "+ model_dir + "/part_meshes && ls *.obj | cut -f1 -d'.' > partids.txt && parallel --colsep=',' -j 4 --eta 'assimp export {1}.obj {1}.dae' :::: partids.txt")
        
        object_id = model_dir.split("/")[-1]

        #json file is used to get the relationship between the link and the meshes
        file = open(f'{dir}/{object_id}/{object_id}.articulated-parts.json')
        json_data = json.load(file)
        file.close()

        #articulations information 
        articulations = json_data['articulations']

        #parts information
        parts = json_data['parts']

        #add a virtual part for the following tree 0: unknown the virtual point
        part_num = len(parts) 
        
        #construct the tree based on the childId
        parent = np.zeros(part_num, dtype=int)
        for i in range(part_num):
            parent[i] = -1
        children = {}

        #get the information of the articualtions
        joint_type = np.zeros(part_num, dtype=int) #0:fixed, 1:"Central Rotation and Hinge Rotation", 2:"Translation"
        origin = np.zeros((part_num, 3))
        axis = np.zeros((part_num, 3))
        rangeMin = np.zeros(part_num)
        rangeMax = np.zeros(part_num)
        name = {}

        for i in parts:
            if i != None:
                part_id = i['pid']

                #get the children and parent information
                if 'childIds' in i.keys():
                    children[part_id] = i['childIds']
                    for j in children[part_id]:
                        parent[j] = part_id

                #get the name information
                name[part_id] = i['name']

                #get the articulation information
                if 'articulationIds' in i.keys():
                    arti_id = i['articulationIds'][0]
                    origin[part_id] = articulations[arti_id]['origin']
                    axis[part_id] = articulations[arti_id]['axis']
                    rangeMin[part_id] = articulations[arti_id]['rangeMin']
                    rangeMax[part_id] = articulations[arti_id]['rangeMax']
                    if articulations[arti_id]['type'] == 'Translation':
                        joint_type[part_id] = 2
                    else:
                        joint_type[part_id] = 1

        children[0] = []
        for i in range(1, part_num):
            if parent[i] == -1:
                parent[i] = 0
                children[0].append(i)
        name[0] = 'virtual'
        origin[0] = [0, 0, 0]
        
        #used to write the URDF
        URDF = []
        URDF.append(f'<robot name="{object_id}">')
        URDF.append('\n')

        #used to add all the links
        for i in range(part_num):
            if i != 0:
                temp_URDF = add_link(name[i], i, origin[i], dir)
                URDF.extend(temp_URDF)  
            else:
                URDF.append(f'\t<link name="{i}:{name[i]}">')
                URDF.append(f'\t\t<visual>')
                URDF.append(f'\t\t\t<origin xyz="0.0 0.0 0.0" rpy="0 0 0"/>')
                URDF.append(f'\t\t\t<geometry>')
                URDF.append(f'\t\t\t\t<cylinder length="0" radius="0"/>')
                URDF.append(f'\t\t\t</geometry>')
                URDF.append(f'\t\t</visual>')
                URDF.append(f'\t</link>')
                URDF.append(f'\n')

        for i in parts:
            if i != None:
                part_id = i['pid']
                par = parent[part_id]
                if 'articulationIds' in i.keys():
                    temp_URDF = add_joint(joint_type[part_id], str(par)+':'+name[par], str(part_id)+':'+name[part_id], origin[part_id]-origin[par], axis[part_id], rangeMin[part_id], rangeMax[part_id])
                    URDF.extend(temp_URDF)
                else:
                    temp_URDF = add_joint(joint_type[part_id], str(par)+':'+name[par], str(part_id)+':'+name[part_id], origin[part_id]-origin[par])
                    URDF.extend(temp_URDF)

        URDF.append(f'</robot>')

        file = open(f'URDF/{object_id}.URDF', 'w')
        file.write('\n'.join(URDF))
        file.close()




