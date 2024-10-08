from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.ext.declarative import declarative_base

URL_DATABASE = 'postgresql://postgres:Dima%402020@localhost:5432/VRP_Routes_db'

engine = create_engine(URL_DATABASE)

SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

Base = declarative_base()

def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

{
    "routes":
        [{
            "id":0,
            "no_of_oders" : 3,
            "total_weigt" : 20,
            "total_distance": 240000,
            "total_time" : 230,
            "route" :
            [{
                "id":0,
                "no_of_oders_for_route" : 3,
                "total_weight_for_route" : 20,
                "total_distancle_for_route": 240000,
                "total_time_for_route" : 230,
                "vehicle_id" : 24567,
                "orders":
                [
                    {
                        "id" : 0,
                        "order_id" :1,
                        "order_weight" : 2,
                        "latitude" : "6.1178564993217",
                        "longitude" : "81.06391786122536",
                        "arrive_time" : "8:40AM-9:00AM"
                    },
                    {
                        "id" : 1,
                        "order_id" :2,
                        "order_weight" : 2,
                        "latitude" : "6.1178564993217",
                        "longitude" : "81.06391786122536",
                        "arrive_time" : "8:40AM-9:00AM"
                    },
                    {
                        "id" : 2,
                        "order_id" :3,
                        "order_weight" : 2,
                        "latitude" : "6.1178564993217",
                        "longitude" : "81.06391786122536",
                        "arrive_time" : "8:40AM-9:00AM"
                    }
                ]
            }]
        }]
}