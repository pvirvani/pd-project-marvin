from flask import Flask, jsonify
from flask_cors import CORS
from flask import Flask, jsonify, request
import json
import os
import uuid


# configuration
DEBUG = True

# instantiate the app
app = Flask(__name__)
app.config.from_object(__name__)

# enable CORS
CORS(app, resources={r'/*': {'origins': '*'}})
##########################
# For Gloabl Data
##########################

# for global projects
if os.path.exists('./projects.json'):
        with open('projects.json', 'r') as fprojects:
            PROJECTS = json.load(fprojects)
else:
        PROJECTS=[]

# for global archives
if os.path.exists('./archive.json'):
        with open('archive.json', 'r') as ftrash:
            Archive = json.load(ftrash)
else:
        Archive=[]


# ######### if you need global actions ###########
# 
# But currently we will not be using global actions
# as each demo of each project's will have its own
# actions
# 
# ################################################
# #  for global actions
# if os.path.exists('./actions.json'):
#         with open('actions.json', 'r') as faction:
#             Actions = json.load(faction)
# else:
#         Actions=[]


# ######################


# sanity check route
# @app.route('/ping', methods=['GET'])
# def ping_pong():
#     return jsonify('pong!')

# @app.route('/projects', methods=['GET'])
# def all_projects():
#     return jsonify({
#         'status': 'success',
#         'projects': PROJECTS
#     })

""" def generate_sequence(actions):
    output = []
    for action in actions:
        action_params = json.loads(action['action_parameters'])
        color = action_params['color']
        lego = action_params['lego']
        lego_id = action_params['id']
        pickedfrom = action_params['pickedfrom']
        placedat = action_params['placedat']
        
        
        # Process pickedfrom
        pickedfrom_str = ':'.join(str(item) for item in pickedfrom)
        pickedfrom_str = pickedfrom_str.replace("_", "")

        # Process placedat
        placedat_str = ':'.join(str(item) for item in placedat)
        placedat_str = placedat_str.replace("_", "")
        # print(pickedfrom_str,placedat_str)

        # Generate pick action
        pick_action = f"pick({color}_{lego}_{lego_id}_{pickedfrom_str})"
        output.append(pick_action)

        # Generate place or stack action
        if len(placedat) == 4:
            x = int(placedat[2])
            if x > 1:
                stack_action = f"stack({color}_{lego}_{lego_id}_{pickedfrom_str},{color}_{lego}_{lego_id}_{placedat_str})"
                output.append(stack_action)
            else:
                place_action = f"place({color}_{lego}_{lego_id}_{placedat_str})"
                output.append(place_action)
        elif len(placedat) == 2:
            x = int(placedat[1])
            if x > 1:
                stack_action = f"stack({color}_{lego}_{lego_id}_{pickedfrom_str},{color}_{lego}_{lego_id}_{placedat_str})"
                output.append(stack_action)
            else:
                place_action = f"place({color}_{lego}_{lego_id}_{placedat_str})"
                output.append(place_action)
    
    #return '\n'.join(output)
    return output """

def generate_sequence(actions):
    output = []
    for action in actions:
        action_params = json.loads(action['action_parameters'])
        color = action_params['color']
        lego = action_params['lego']
        lego_id = action_params['id']
        pickedfrom = action_params['pickedfrom']
        placedat = action_params['placedat']
        
        
        # Process pickedfrom
        pickedfrom_str = ':'.join(str(item) for item in pickedfrom)
        pickedfrom_str = pickedfrom_str.replace("_", "")

        # Process placedat
        placedat_str = ':'.join(str(item) for item in placedat)
        placedat_str = placedat_str.replace("_", "")
        # print(pickedfrom_str,placedat_str)

        # Generate pick action
        pick_action = f"pick({color}_{lego}_{lego_id}_{pickedfrom_str})"
        output.append(pick_action)

        # Generate place or stack action
        if len(placedat) == 4:
            x = int(placedat[2])
            if x > 1:
                stack_action = f"stack({color}_{lego}_{lego_id}_{pickedfrom_str},{color}_{lego}_{lego_id}_{placedat_str})"
                output.append(stack_action)
            else:
                place_action = f"place({color}_{lego}_{lego_id}_{placedat_str})"
                output.append(place_action)
        elif len(placedat) == 2:
            x = int(placedat[1])
            if x > 1:
                stack_action = f"stack({color}_{lego}_{lego_id}_{pickedfrom_str},{color}_{lego}_{lego_id}_{placedat_str})"
                output.append(stack_action)
            else:
                place_action = f"place({color}_{lego}_{lego_id}_{placedat_str})"
                output.append(place_action)
    
    # return '\n'.join(output)
    return output


##################################### Projects APIs ###########################
@app.route('/projects', methods=['GET', 'POST'])
def all_projects():
    
    response_object = {'status': 'success'}
    if request.method == 'POST':
        post_data = request.get_json()
        proj_id = uuid.uuid4().hex
        PROJECTS.append({
            'p_id': proj_id,
            'p_name': post_data.get('p_name'),
            'p_problems': post_data.get('p_problems'),
            'p_actions': post_data.get('p_actions')
        })
        with open('projects.json', 'w') as fp:
            json.dump(PROJECTS, fp)
        os.mkdir(os.path.join('projects',post_data.get('p_name')+'_'+proj_id))
        response_object['message'] = 'Project Created'
    else:
        response_object['projects'] = PROJECTS

    
    return jsonify(response_object)

# def remove_project(project_id):
#     for p in PROJECTS:
#         if p['id'] == project_id:
#             PROJECTS.remove(p)
#             return True
#     return False

@app.route('/updateproject', methods=['GET', 'POST'])
def single_project():
    response_object = {'status': 'success'}
    post_data = request.get_json()
    proj_id = post_data.get('p_id')
    for projs in PROJECTS:
        for k, v in projs.items():
            if v == proj_id:
                old_proj_name = projs['p_name'] 
                projs.update({
                    'p_name': post_data.get('p_name'),
                    # 'p_problems': post_data.get('p_problems'),
                    # 'p_actions': post_data.get('p_actions')
                })
    with open('projects.json', 'w') as fp:
        json.dump(PROJECTS, fp)
    os.rename(os.path.join('projects',old_proj_name+'_'+proj_id),os.path.join('projects',post_data.get('p_name')+'_'+proj_id))
    response_object['message'] = 'Project Updated!'
    response_object['projects'] = PROJECTS
    return jsonify(response_object)

@app.route('/deleteproject', methods=['GET', 'POST'])
def delete_project():
    response_object = {'status': 'success'}
    post_data = request.get_json()
    proj_id = post_data.get('p_id')
    for projs in PROJECTS:
        for k, v in projs.items():
            if v == proj_id:
                proj_name = projs['p_name']
                Archive.append(projs)
                PROJECTS.remove(projs)
    with open('projects.json', 'w') as fp:
        json.dump(PROJECTS, fp)
    with open('archive.json', 'w') as fp:
        json.dump(Archive, fp)
    # os.mkdir(os.path.join('bin',proj_name+'_'+proj_id))
    # os.rmdir(os.path.join('projects',proj_name+'_'+proj_id))
    response_object['message'] = 'Project Removed!'
    response_object['projects'] = PROJECTS

    return jsonify(response_object)

@app.route('/copyproject', methods=['GET', 'POST'])
def copy_project():
    response_object = {'status': 'success'}
    post_data = request.get_json()
    old_proj_id = post_data.get('p_id')
    proj_id =uuid.uuid4().hex
    for projs in PROJECTS:
        for k, v in projs.items():
            if v == old_proj_id:
                PROJECTS.append({
                    'p_id': proj_id,
                    'p_name': post_data.get('p_name')+'-copy',
                    'p_problems': post_data.get('p_problems'),
                    'p_actions': post_data.get('p_actions')
                })
    with open('projects.json', 'w') as fp:
        json.dump(PROJECTS, fp)
    os.mkdir(os.path.join('projects',post_data.get('p_name')+'-copy'+'_'+proj_id))
    response_object['message'] = 'Project Updated!'
    response_object['projects'] = PROJECTS
    return jsonify(response_object)

######################################## Demonstrations APIs ##################################

@app.route('/getprojectdata', methods=['GET', 'POST'])
def getProjectData():
    
    response_object = {'status': 'success'}
    post_data = request.get_json()
    proj_parent_id = post_data.get('parent_id')
    for projs in PROJECTS:
        for k, v in projs.items():
            if v == proj_parent_id:
                proj_parent_name = projs['p_name']
        response_object['message'] = 'Data Retrieved'
    else:
        response_object['projdata'] = proj_parent_name

    
    return jsonify(response_object)

@app.route('/getdemos', methods=['GET', 'POST'])
def getDemos():
    demos = []
    response_object = {'status': 'success'}
    post_data = request.get_json()
    proj_parent_id = post_data.get('parent_id')
    proj_parent_name = post_data.get('parent_name')
    if os.path.exists(os.path.join('projects',proj_parent_name+'_'+proj_parent_id,'demos.json')):
        with open(os.path.join('projects',proj_parent_name+'_'+proj_parent_id,'demos.json'),'r') as dfile:
            demos = json.load(dfile)        
    else:
        response_object['message'] = 'Demos Empty'

    response_object['demos'] = demos
    return jsonify(response_object)

@app.route('/adddemo', methods=['GET', 'POST'])
def addDemo():
    demos = []
    response_object = {'status': 'success'}
    post_data = request.get_json()
    proj_parent_id = post_data.get('parent_id')
    proj_parent_name = post_data.get('parent_name')
    demo_id = uuid.uuid4().hex
    if os.path.exists(os.path.join('projects',proj_parent_name+'_'+proj_parent_id,'demos.json')):
        with open(os.path.join('projects',proj_parent_name+'_'+proj_parent_id,'demos.json'),'r') as dfile:
            demos = json.load(dfile)
        demos.append({
            'parent_id': proj_parent_id,
            'parent_name': proj_parent_name,
            'demo_id': demo_id,
            'demo_name': post_data.get('d_name'),
            'actions':[],
            'demo_actions': 0
        })
        with open(os.path.join('projects',proj_parent_name+'_'+proj_parent_id,'demos.json'), 'w') as fp:
            json.dump(demos, fp)
             
    else:
        demos.append({
            'parent_id': proj_parent_id,
            'parent_name': proj_parent_name,
            'demo_id': demo_id,
            'demo_name': post_data.get('d_name'),
            'actions':[],
            'demo_actions': 0
        })
        with open(os.path.join('projects',proj_parent_name+'_'+proj_parent_id,'demos.json'), 'w') as fp:
            json.dump(demos, fp)

    response_object['demos'] = demos
    return jsonify(response_object)

@app.route('/deletedemo', methods=['GET', 'POST'])
def deleteDemo():
    response_object = {'status': 'success'}
    post_data = request.get_json()
    proj_parent_id = post_data.get('parent_id')
    proj_parent_name = post_data.get('parent_name')
    demo_id = post_data.get('demo_id')
    
    with open(os.path.join('projects',proj_parent_name+'_'+proj_parent_id,'demos.json'),'r') as dfile:
        demos = json.load(dfile)
    for demo in demos:
        for k, v in demo.items():
            if v == demo_id:
                demos.remove(demo)
    with open(os.path.join('projects',proj_parent_name+'_'+proj_parent_id,'demos.json'), 'w') as fp:
        json.dump(demos, fp)
    response_object['demos'] = demos
    return jsonify(response_object)

@app.route('/copydemo', methods=['GET', 'POST'])
def copyDemo():
    response_object = {'status': 'success'}
    post_data = request.get_json()
    proj_parent_id = post_data.get('parent_id')
    proj_parent_name = post_data.get('parent_name')
    old_demo_name = post_data.get('demo_name')
    demo_id = uuid.uuid4().hex
    with open(os.path.join('projects',proj_parent_name+'_'+proj_parent_id,'demos.json'),'r') as dfile:
        demos = json.load(dfile)
    demos.append({
        'parent_id': proj_parent_id,
        'parent_name': proj_parent_name,
        'demo_id': demo_id,
        'demo_name': old_demo_name+'-copy',
        'actions': [],
        'demo_actions': 0
    })
    with open(os.path.join('projects',proj_parent_name+'_'+proj_parent_id,'demos.json'), 'w') as fp:
        json.dump(demos, fp)
            

    response_object['demos'] = demos
    return jsonify(response_object)

@app.route('/updatedemo', methods=['GET', 'POST'])
def updateDemo():
    response_object = {'status': 'success'}
    post_data = request.get_json()
    proj_parent_id = post_data.get('parent_id')
    proj_parent_name = post_data.get('parent_name')
    old_demo_id = post_data.get('demo_id')
    with open(os.path.join('projects',proj_parent_name+'_'+proj_parent_id,'demos.json'),'r') as dfile:
        demos = json.load(dfile)
    for demo in demos:
        for k, v in demo.items():
            if v == old_demo_id:
                demo.update({
                    'demo_name': post_data.get('demo_name'),
                })
    with open(os.path.join('projects',proj_parent_name+'_'+proj_parent_id,'demos.json'), 'w') as fp:
        json.dump(demos, fp)
            

    response_object['demos'] = demos
    return jsonify(response_object)

######################################## Demonstration: Actions APIs ########################

# # actions are properties of a demo
# @app.route('/addaction', methods=['GET', 'POST'])
# def addAction():
#     response_object = {'status': 'success'}
#     post_data = request.get_json()
#     proj_parent_id = post_data.get('parent_id')
#     proj_parent_name = post_data.get('parent_name')
#     demo_parent_id = post_data.get('demo_id')
#     demo_parent_name =post_data.get('demo_name')
#     action_id = uuid.uuid4().hex
#     if os.path.exists(os.path.join('projects',proj_parent_name+'_'+proj_parent_id,'demos.json')):
#         with open(os.path.join('projects',proj_parent_name+'_'+proj_parent_id,'demos.json'),'r') as dfile:
#             demos = json.load(dfile)
#         for demo in demos:
#             for k, v in demo.items():
#                 if v == demo_parent_id:
#                     demo.append({
#                         'actions':{
#                         'action_id': action_id,
#                         'action_name':post_data.get(action_name),
#                         'action_parameters':post_data.get(action_parameters)
#                         },
#                         # 'demo_actions': demo.actions.length()+1
#                     })
#                     with open(os.path.join('projects',proj_parent_name+'_'+proj_parent_id,'demos.json'), 'w') as fp:
#                         json.dump(demos, fp)
             
#     # else:
#     #     demos.append({
#     #         'parent_id': proj_parent_id,
#     #         'parent_name': proj_parent_name,
#     #         'demo_id': demo_parent_id,
#     #         'demo_name': post_data.get('d_name'),
#     #         'demo_actions': 0
#     #     })
#         with open(os.path.join('projects',proj_parent_name+'_'+proj_parent_id,'demos.json'), 'w') as fp:
#             json.dump(demos, fp)

#     response_object['demos'] = demos
#     return jsonify(response_object)




# @app.route('/actions', methods=['GET', 'POST'])
# def all_actions():
    
#     response_object = {'status': 'success'}
#     if request.method == 'POST':
#         post_data = request.get_json()
#         action_id = uuid.uuid4().hex
#         Actions.append({
#             'action_id': action_id,
#             'action_name': post_data.get('action_name'),
#             'action_parameters':post_data.get('action_parameters')
#         })
#         with open('actions.json', 'w') as fp:
#             json.dump(Actions, fp)
#         # os.mkdir(os.path.join('projects',post_data.get('p_name')+'_'+action_id))
#         response_object['message'] = 'Action Created'
#     else:
#         response_object['actions'] = Actions

#     return jsonify(response_object)

# @app.route('/deleteaction', methods=['GET', 'POST'])
# def deleteAction():
#     response_object = {'status': 'success'}
#     post_data = request.get_json()
#     proj_parent_id = post_data.get('parent_id')
#     proj_parent_name = post_data.get('parent_name')
#     action_id = post_data.get('action_id')
    
#     with open('actions.json','r') as afile:
#         actions = json.load(afile)
#     for action in actions:
#         for k, v in action.items():
#             if v == action_id:
#                 actions.remove(action)
#     with open('actions.json','w') as fp:
#         json.dump(actions, fp)
#     response_object['actions'] = actions
#     return jsonify(response_object)


# -------------- adding action in nested dictionary of demos
@app.route('/getactions', methods=['GET','POST'])
def get_actions():
    demos=[]
    response_object = {'status': 'success'}
    post_data = request.get_json()
    proj_parent_id = post_data.get('project_id')
    proj_parent_name = post_data.get('project_name')
    demo_parent_id = post_data.get('demo_id')
    if os.path.exists(os.path.join('projects',proj_parent_name+'_'+proj_parent_id,'demos.json')):
        with open(os.path.join('projects',proj_parent_name+'_'+proj_parent_id,'demos.json'),'r') as dfile:
            demos = json.load(dfile)
        for demo in demos:
            for k, v in demo.items():
                if v == demo_parent_id:
                    response_object['actions']=demo['actions']
    return jsonify(response_object)
    # return jsonify(response_object)

@app.route('/getsequence', methods=['GET','POST'])
def get_sequence():
    demos=[]
    response_object = {'status': 'success'}
    post_data = request.get_json()
    proj_parent_id = post_data.get('project_id')
    proj_parent_name = post_data.get('project_name')
    demo_parent_id = post_data.get('demo_id')
    if os.path.exists(os.path.join('projects',proj_parent_name+'_'+proj_parent_id,'demos.json')):
        with open(os.path.join('projects',proj_parent_name+'_'+proj_parent_id,'demos.json'),'r') as dfile:
            demos = json.load(dfile)
        for demo in demos:
            for k, v in demo.items():
                if v == demo_parent_id:
                    response_object['action_sequence']=demo['action_sequence']
    return jsonify(response_object)
    # return jsonify(response_object)

@app.route('/addactions', methods=['POST'])
def add_actions():
    demos=[]
    response_object = {'status': 'success'}
    post_data = request.get_json()
    proj_parent_id = post_data.get('project_id')
    proj_parent_name = post_data.get('project_name')
    demo_parent_id = post_data.get('demo_id')
    demo_parent_name = post_data.get('demo_name')
    action_id = uuid.uuid4().hex
    if os.path.exists(os.path.join('projects',proj_parent_name+'_'+proj_parent_id,'demos.json')):
        with open(os.path.join('projects',proj_parent_name+'_'+proj_parent_id,'demos.json'),'r') as dfile:
            demos = json.load(dfile)
        for demo in demos:
            for k,v in demo.items():
                if v == demo_parent_id:
                    demo['actions'].append({
                        'action_id':action_id,
                        'action_name':post_data.get('action_name'),
                        'action_parameters':post_data.get('action_parameters')
                    })
                    demo['demo_actions'] = demo['demo_actions']+1
    with open(os.path.join('projects',proj_parent_name+'_'+proj_parent_id,'demos.json'), 'w') as fp:
        json.dump(demos, fp)
    # os.mkdir(os.path.join('projects',post_data.get('p_name')+'_'+action_id))
    response_object['message'] = 'Action Created'
    return jsonify(response_object)

@app.route('/deleteaction', methods=['GET', 'POST'])
def deleteAction():
    response_object = {'status': 'success'}
    post_data = request.get_json()
    proj_parent_id = post_data.get('parent_id')
    proj_parent_name = post_data.get('parent_name')
    demo_id = post_data.get('demo_id')
    action_id = post_data.get('action_id')
    
    with open(os.path.join('projects',proj_parent_name+'_'+proj_parent_id,'demos.json'),'r') as dfile:
        demos = json.load(dfile)
    for demo in demos:
        for k, v in demo.items():
            if v == demo_id:
                actions = demo.get("actions")
                for action in actions:
                    if action.get("action_id") == action_id:
                        actions.remove(action)
    with open(os.path.join('projects',proj_parent_name+'_'+proj_parent_id,'demos.json'), 'w') as fp:
        json.dump(demos, fp)
    response_object['demos'] = demos
    return jsonify(response_object)


@app.route('/updateaction', methods=['GET', 'POST'])
def updateAction():
    response_object = {'status': 'success'}
    post_data = request.get_json()
    proj_parent_id = post_data.get('parent_id')
    proj_parent_name = post_data.get('parent_name')
    demo_id = post_data.get('demo_id')
    action_id = post_data.get('action_id')
    new_action_name = post_data.get('action_name')
    with open(os.path.join('projects',proj_parent_name+'_'+proj_parent_id,'demos.json'),'r') as dfile:
        demos = json.load(dfile)
    for demo in demos:
        for k, v in demo.items():
            if v == demo_id:
                actions = demo.get("actions")
                for action in actions:
                    if action.get("action_id") == action_id:
                        action["action_name"]= new_action_name
                        # actions.remove(action)
    with open(os.path.join('projects',proj_parent_name+'_'+proj_parent_id,'demos.json'), 'w') as fp:
        json.dump(demos, fp)
            

    response_object['demos'] = demos
    return jsonify(response_object)


# @app.route('/generatesequence', methods=['GET', 'POST'])
# def generateSequence():
#     response_object = {'status': 'success'}
#     post_data = request.get_json()
#     proj_parent_id = post_data.get('parent_id')
#     proj_parent_name = post_data.get('parent_name')
#     demo_id = post_data.get('demo_id')
#     with open(os.path.join('projects',proj_parent_name+'_'+proj_parent_id,'demos.json'),'r') as dfile:
#         demos = json.load(dfile)
#     for demo in demos:
#         for k, v in demo.items():
#             if v == demo_id:
#                 actions = demo.get("actions")
#                 sequence = generate_sequence(actions)
    
#     response_object['sequence'] = sequence
#     return jsonify(response_object)
# 
@app.route('/generatesequence', methods=['GET', 'POST'])
def generateSequence():
    response_object = {'status': 'success'}
    post_data = request.get_json()
    proj_parent_id = post_data.get('parent_id')
    proj_parent_name = post_data.get('parent_name')
    demo_id = post_data.get('demo_id')
    with open(os.path.join('projects',proj_parent_name+'_'+proj_parent_id,'demos.json'),'r') as dfile:
        demos = json.load(dfile)
    for demo in demos:
        for k, v in demo.items():
            if v == demo_id:
                actions = demo.get("actions")
                sequence = generate_sequence(actions)
    
    output_dict = {'action_sequence': sequence}
    demo.update(output_dict)
    with open(os.path.join('projects',proj_parent_name+'_'+proj_parent_id,'demos.json'), 'w') as fp:
        json.dump(demos, fp)

    # Convert the dictionary to a JSON string with indentation
    response_object['demos'] = demos
    return jsonify(response_object)                   

############################### Problems APIs ####################################


if __name__ == '__main__':
    app.run()