from sqlalchemy import Boolean, Column, ForeignKey, Integer, String, Float
from sqlalchemy.orm import relationship
from app.database.database import Base

# Model for the main routes
class Routes(Base):
    __tablename__ = 'routes'
    
    id = Column(Integer, primary_key=True, index=True)
    # no_of_orders = Column(Integer, nullable=False)
    total_weight = Column(Float, nullable=False)
    total_distance = Column(Float, nullable=False)
    total_time = Column(Float, nullable=False)
    # optimized_type = Column(String, nullable=True)
    
    # One-to-many relationship with VehicleRoute
    vehicle_routes = relationship("VehicleRoute", back_populates="route", cascade="all, delete-orphan")


# Model for detailed information about individual vehicle route segments
class VehicleRoute(Base):
    __tablename__ = 'vehicle_routes'
    
    id = Column(Integer, primary_key=True, index=True)
    routes_id = Column(Integer, ForeignKey("routes.id"), nullable=False)  # Corrected the foreign key
    vehicle_id = Column(Integer, nullable=False)
    no_of_orders_for_route = Column(Integer, nullable=False)
    total_weight_for_route = Column(Float, nullable=False)
    total_distance_for_route = Column(Float, nullable=False)
    total_time_for_route = Column(Float, nullable=False)
    
    # Relationship with orders
    orders = relationship("Orders", back_populates="vehicle_route", cascade="all, delete-orphan")
    
    # Back reference to Routes
    route = relationship("Routes", back_populates="vehicle_routes")


# Model for orders assigned to specific route segments
class Orders(Base):
    __tablename__ = 'orders'
    
    id = Column(Integer, primary_key=True, index=True)
    order_id = Column(Integer, index=True)
    order_weight = Column(Float, index=True)
    latitude = Column(String, index=True)
    longitude = Column(String, index=True)
    arrive_time = Column(String, nullable=True)
    
    # ForeignKey linking to VehicleRoute
    vehicle_route_id = Column(Integer, ForeignKey("vehicle_routes.id"), nullable=False)  # Corrected foreign key
    
    # Back reference to VehicleRoute
    vehicle_route = relationship("VehicleRoute", back_populates="orders")