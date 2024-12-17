import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset
from sklearn.model_selection import train_test_split
import os


class GraspingPoseEval(nn.Module):

    # ! DEFINE NEURAL NETWORK MODEL

    def __init__(self, input_size, hidden_size, output_size):
        super(GraspingPoseEval, self).__init__()
        self.fc1 = nn.Linear(input_size, hidden_size)
        self.relu = nn.ReLU()
        self.fc2 = nn.Linear(hidden_size, output_size)
        self.sigmoid = nn.Sigmoid()

    def forward(self, x):
        x = self.fc1(x)
        x = self.relu(x)
        x = self.fc2(x)
        x = self.sigmoid(x)
        return x


# ! LOAD AND PREPROCESS DATA

# Load in data (X: grasping poses, y: labels)
X = []
y = []
# Dynamically get the home directory
home_dir = os.path.expanduser("~/WorkSpace/part_code")
# Construct the desired path relative to the home directory
pose_path = os.path.join(
    home_dir,
    # "dev",
    "storage",
    "train_data.txt",
)
result_path = os.path.join(
    home_dir,
    # "dev",
    "storage",
    "results.txt",
)
for line in open(pose_path, "r"):
    pose = [float(i) for i in line.split()]
    X.append(pose)

for val in open(result_path, "r"):
    y.append(int(val))

X_train, X_val, y_train, y_val = train_test_split(X, y, test_size=0.2, random_state=42)

# Convert to PyTorch tensors
X_train_tensor = torch.FloatTensor(X_train)
y_train_tensor = torch.FloatTensor(y_train).view(-1, 1)

X_val_tensor = torch.FloatTensor(X_val)
y_val_tensor = torch.FloatTensor(y_val).view(-1, 1)

# Create DataLoader
train_dataset = TensorDataset(X_train_tensor, y_train_tensor)
train_loader = DataLoader(train_dataset, batch_size=32, shuffle=True)

val_dataset = TensorDataset(X_val_tensor, y_val_tensor)
val_loader = DataLoader(val_dataset, batch_size=32, shuffle=False)

# ! INITIALIZE MODEL AND OPTIMIZER

input_size = len(X_train[0])
hidden_size = 10
output_size = 1

model = GraspingPoseEval(input_size, hidden_size, output_size)
criterion = nn.BCELoss()
optimizer = optim.Adam(model.parameters(), lr=0.001)

# ! TRAIN MODEL

num_epochs = 2000

for epoch in range(num_epochs):
    model.train()
    for inputs, labels in train_loader:
        optimizer.zero_grad()
        outputs = model(inputs)
        loss = criterion(outputs, labels)
        loss.backward()
        optimizer.step()

    # Validation
    model.eval()
    with torch.no_grad():
        val_outputs = model(X_val_tensor)
        val_loss = criterion(val_outputs, y_val_tensor)

    print(
        f"Epoch [{epoch+1}/{num_epochs}], Loss: {loss.item():.4f}, Val Loss: {val_loss.item():.4f}"
    )


# ! SAVE MODEL

torch.save(model.state_dict(), "new_model.pt")
