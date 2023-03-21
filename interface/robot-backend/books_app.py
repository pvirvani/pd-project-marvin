from flask import Flask, jsonify
from flask_cors import CORS
from flask import Flask, jsonify, request
import json
import os


# configuration
DEBUG = True

# instantiate the app
app = Flask(__name__)
app.config.from_object(__name__)

# enable CORS
CORS(app, resources={r'/*': {'origins': '*'}})

# sanity check route
@app.route('/ping', methods=['GET'])
def ping_pong():
    return jsonify('pong!')

# @app.route('/books', methods=['GET'])
# def all_books():
#     return jsonify({
#         'status': 'success',
#         'books': BOOKS
#     })
@app.route('/books', methods=['GET', 'POST'])
def all_books():
    if os.path.exists('./project_file/projects.json'):
        with open('project_file/projects.json', 'r') as fprojects:
            BOOKS = json.load(fprojects)
    else:
        BOOKS=[]
    response_object = {'status': 'success'}
    if request.method == 'POST':
        post_data = request.get_json()
        BOOKS.append({
            'title': post_data.get('title'),
            'author': post_data.get('author'),
            'author': post_data.get('author'),
            # 'read': post_data.get('read')
        })
        with open('project_file/projects.json', 'w') as fp:
            json.dump(BOOKS, fp)
        response_object['message'] = 'Book added!'
    else:
        response_object['books'] = BOOKS

    
    return jsonify(response_object)


if __name__ == '__main__':
    app.run()