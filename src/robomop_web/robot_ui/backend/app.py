from flask import Flask
from flask_restful import Api
from flask_cors import CORS
from models import db, Schedule, Setting
from resources import ScheduleResource, SettingResource

app = Flask(__name__)
CORS(app)
api = Api(app)

app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///database.db'
db.init_app(app)

with app.app_context():
    db.create_all()

api.add_resource(ScheduleResource, '/api/schedules', '/api/schedules/<int:schedule_id>')
api.add_resource(SettingResource, '/api/settings')

if __name__ == '__main__':
    app.run(debug=True)
