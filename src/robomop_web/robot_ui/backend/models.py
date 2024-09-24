from flask_sqlalchemy import SQLAlchemy

db = SQLAlchemy()

class Schedule(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    time = db.Column(db.String(50))
    days = db.Column(db.String(100))

class Setting(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    cleaning_mode = db.Column(db.String(50))
    volume = db.Column(db.Integer)
    notifications = db.Column(db.Boolean)
