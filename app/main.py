from fastapi import FastAPI
from app.routes import optimize  # Import the optimize router
from app.database.database import engine
from app.models import models

app = FastAPI()

models.Base.metadata.create_all(bind=engine)

# Include the router for the route optimization functionality
app.include_router(optimize.router, prefix="/api/route_optimize", tags=["optimize"])

# Now, the /optimize_route/ will be available at: /api/v1/optimize_route