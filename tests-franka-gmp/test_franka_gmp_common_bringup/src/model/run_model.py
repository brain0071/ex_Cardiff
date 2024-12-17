import torch
from train_model import GraspingPoseEval

input_size = 11
hidden_size = 10
output_size = 1

model = GraspingPoseEval(input_size, hidden_size, output_size)
model.load_state_dict(torch.load("new_model.pt"))

# ! PREDICTION AND RANKING

new_poses = []
for line in open("/home/naodai/WorkSpace/part_code/storage/train_data.txt"):
    pose = [float(i) for i in line.split()]
    new_poses.append(pose)

model.eval()
with torch.no_grad():
    new_poses_tensor = torch.FloatTensor(new_poses)
    predictions = model(new_poses_tensor)

# Rank based on predictions
ml_ranking = torch.argsort(predictions.squeeze(), descending=True)

# Save ranking
ml_array = ml_ranking.numpy()
ml_list = list(ml_array)
ml_list_formatted = []
for i in ml_list:
    ml_list_formatted.append(i)
   
print(ml_list_formatted[0])
print("choose the ", ml_list_formatted[0], " grasp point:")
print(new_poses[0][0:7])
