from fastapi import FastAPI
from pydantic import BaseModel
import uvicorn

app = FastAPI()

class Query(BaseModel):
    text: str

@app.post("/parse")
def parse(q: Query):
    if "去厨房" in q.text:
        return {
            "tasks": [
                {"type": "navigate", "pose": {"x": 2.0, "y": 1.0, "theta": 0.0}},
                {"type": "observe"},
                {"type": "navigate", "pose": {"x": 0.0, "y": 0.0, "theta": 0.0}}
            ]
        }
    return {"tasks": []}

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
