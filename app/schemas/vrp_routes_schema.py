from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from app.models.models import models
from app.database.database import get_db
from typing import List
from pydantic import BaseModel

# Create the API Router
router = APIRouter()

# Pydantic models for request and response schemas
class OrderBase(BaseModel):
    order_id: int
    order_weight: float
    latitude: str
    longitude: str
    arrive_time: str

class RouteDetailBase(BaseModel):
    no_of_orders_for_route: int
    total_weight_for_route: float
    total_distance_for_route: float
    total_time_for_route: int
    vehicle_id: int
    orders: List[OrderBase]

class RouteBase(BaseModel):
    no_of_orders: int
    total_weight: float
    total_distance: float
    total_time: int
    route: List[RouteDetailBase]