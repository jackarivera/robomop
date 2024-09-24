from flask import request
from flask_restful import Resource
from models import db, Schedule, Setting

class ScheduleResource(Resource):
    def get(self, schedule_id=None):
        if schedule_id:
            schedule = Schedule.query.get(schedule_id)
            if schedule:
                return {
                    'id': schedule.id,
                    'time': schedule.time,
                    'days': schedule.days.split(',')
                }, 200
            else:
                return {'message': 'Schedule not found'}, 404
        else:
            schedules = Schedule.query.all()
            return [{
                'id': s.id,
                'time': s.time,
                'days': s.days.split(',')
            } for s in schedules], 200

    def post(self):
        data = request.get_json()
        new_schedule = Schedule(
            time=data['time'],
            days=','.join(data['days'])
        )
        db.session.add(new_schedule)
        db.session.commit()
        return {'message': 'Schedule created'}, 201

    def delete(self, schedule_id):
        schedule = Schedule.query.get(schedule_id)
        if schedule:
            db.session.delete(schedule)
            db.session.commit()
            return {'message': 'Schedule deleted'}, 200
        else:
            return {'message': 'Schedule not found'}, 404

class SettingResource(Resource):
    def get(self):
        setting = Setting.query.first()
        if setting:
            return {
                'cleaning_mode': setting.cleaning_mode,
                'volume': setting.volume,
                'notifications': setting.notifications
            }, 200
        else:
            # Return default settings if none are set
            return {
                'cleaning_mode': 'Standard',
                'volume': 50,
                'notifications': True
            }, 200

    def post(self):
        data = request.get_json()
        setting = Setting.query.first()
        if setting:
            setting.cleaning_mode = data['cleaning_mode']
            setting.volume = data['volume']
            setting.notifications = data['notifications']
        else:
            setting = Setting(
                cleaning_mode=data['cleaning_mode'],
                volume=data['volume'],
                notifications=data['notifications']
            )
            db.session.add(setting)
        db.session.commit()
        return {'message': 'Settings updated'}, 200
