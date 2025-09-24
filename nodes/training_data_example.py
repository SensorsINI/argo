#!/usr/bin/env python3
"""
Example script demonstrating how to load and use training data collected
by the refactored control.py during human control sessions.

This shows how the collected state-action pairs can be used for:
1. Training neural network controllers
2. Analyzing human control patterns  
3. Creating validation datasets
4. Implementing imitation learning
"""

import json
import numpy as np
import pandas as pd
from pathlib import Path
from typing import List, Dict, Any, Tuple
import matplotlib.pyplot as plt
from dataclasses import dataclass
import argparse

@dataclass
class TrainingDataset:
    """Structured training dataset from human control sessions."""
    states: np.ndarray      # Features: [compass, target, wind_speed, wind_angle, ...]
    actions: np.ndarray     # Labels: [rudder, sail]
    timestamps: np.ndarray  # Relative timestamps within sessions
    session_ids: np.ndarray # Which session each sample came from
    feature_names: List[str]
    
    def __len__(self):
        return len(self.states)
    
    def split(self, train_ratio=0.8) -> Tuple['TrainingDataset', 'TrainingDataset']:
        """Split into training and validation sets."""
        n_train = int(len(self) * train_ratio)
        indices = np.random.permutation(len(self))
        
        train_idx = indices[:n_train]
        val_idx = indices[n_train:]
        
        train_dataset = TrainingDataset(
            states=self.states[train_idx],
            actions=self.actions[train_idx], 
            timestamps=self.timestamps[train_idx],
            session_ids=self.session_ids[train_idx],
            feature_names=self.feature_names
        )
        
        val_dataset = TrainingDataset(
            states=self.states[val_idx],
            actions=self.actions[val_idx],
            timestamps=self.timestamps[val_idx], 
            session_ids=self.session_ids[val_idx],
            feature_names=self.feature_names
        )
        
        return train_dataset, val_dataset

class TrainingDataLoader:
    """Loads and processes training data from control.py data collection."""
    
    def __init__(self, data_dir: str = "training_data"):
        self.data_dir = Path(data_dir)
        
    def load_session(self, session_file: Path) -> Dict[str, Any]:
        """Load a single training session file."""
        with open(session_file, 'r') as f:
            return json.load(f)
    
    def extract_features(self, state: Dict[str, Any]) -> List[float]:
        """Extract numerical features from a state dictionary."""
        features = []
        feature_names = []
        
        # Navigation features
        if state['compass_heading'] is not None:
            features.append(state['compass_heading'])
            feature_names.append('compass_heading')
        else:
            features.append(0.0)
            feature_names.append('compass_heading')
            
        if state['target_heading'] is not None:
            features.append(state['target_heading'])
            feature_names.append('target_heading')
        else:
            features.append(0.0)
            feature_names.append('target_heading')
        
        # Heading error (most important for control)
        if state['compass_heading'] is not None and state['target_heading'] is not None:
            # Calculate signed angle difference
            diff = state['target_heading'] - state['compass_heading']
            heading_error = (diff + 180.0) % 360.0 - 180.0
            features.append(heading_error)
            feature_names.append('heading_error')
        else:
            features.append(0.0)
            feature_names.append('heading_error')
        
        # Wind features
        if state['wind_speed'] is not None:
            features.append(state['wind_speed'])
            feature_names.append('wind_speed')
        else:
            features.append(0.0)
            feature_names.append('wind_speed')
            
        if state['wind_angle'] is not None:
            features.append(state['wind_angle'])
            feature_names.append('wind_angle')
        else:
            features.append(0.0)
            feature_names.append('wind_angle')
        
        # GPS features
        if state['gps_sog'] is not None:
            features.append(state['gps_sog'])
            feature_names.append('gps_sog')
        else:
            features.append(0.0)
            feature_names.append('gps_sog')
            
        if state['gps_cog'] is not None:
            features.append(state['gps_cog'])
            feature_names.append('gps_cog')
        else:
            features.append(0.0)
            feature_names.append('gps_cog')
        
        # IMU features (if available)
        if state['gyro'] is not None and isinstance(state['gyro'], dict):
            features.extend([state['gyro']['x'], state['gyro']['y'], state['gyro']['z']])
            feature_names.extend(['gyro_x', 'gyro_y', 'gyro_z'])
        else:
            features.extend([0.0, 0.0, 0.0])
            feature_names.extend(['gyro_x', 'gyro_y', 'gyro_z'])
        
        return features, feature_names
    
    def load_all_sessions(self) -> TrainingDataset:
        """Load all training sessions into a unified dataset."""
        session_files = list(self.data_dir.glob("session_*.json"))
        
        if not session_files:
            raise ValueError(f"No session files found in {self.data_dir}")
        
        all_states = []
        all_actions = []
        all_timestamps = []
        all_session_ids = []
        feature_names = None
        
        print(f"Loading {len(session_files)} training sessions...")
        
        for i, session_file in enumerate(session_files):
            try:
                session_data = self.load_session(session_file)
                print(f"Session {i+1}: {session_data['sample_count']} samples, "
                      f"duration: {session_data['end_time'] - session_data['start_time']:.1f}s")
                
                for sample in session_data['data']:
                    # Extract features
                    features, names = self.extract_features(sample['state'])
                    if feature_names is None:
                        feature_names = names
                    
                    # Extract actions
                    action = sample['action']
                    
                    all_states.append(features)
                    all_actions.append([action['rudder'], action['sail']])
                    all_timestamps.append(sample['relative_time'])
                    all_session_ids.append(i)
                    
            except Exception as e:
                print(f"Error loading {session_file}: {e}")
                continue
        
        if not all_states:
            raise ValueError("No valid training data found")
        
        dataset = TrainingDataset(
            states=np.array(all_states),
            actions=np.array(all_actions),
            timestamps=np.array(all_timestamps),
            session_ids=np.array(all_session_ids),
            feature_names=feature_names
        )
        
        print(f"Loaded {len(dataset)} total samples from {len(session_files)} sessions")
        return dataset

class SimpleNeuralController:
    """Example neural network controller trained on human data."""
    
    def __init__(self, input_size: int):
        # Simple neural network using numpy (for demonstration)
        # In practice, you'd use PyTorch, TensorFlow, etc.
        self.input_size = input_size
        self.hidden_size = 32
        self.output_size = 2  # rudder, sail
        
        # Initialize weights
        self.w1 = np.random.randn(input_size, self.hidden_size) * 0.1
        self.b1 = np.zeros(self.hidden_size)
        self.w2 = np.random.randn(self.hidden_size, self.output_size) * 0.1
        self.b2 = np.zeros(self.output_size)
    
    def forward(self, x):
        """Forward pass through the network."""
        # Hidden layer with ReLU activation
        h = np.maximum(0, np.dot(x, self.w1) + self.b1)
        # Output layer with tanh activation (outputs in [-1, 1])
        y = np.tanh(np.dot(h, self.w2) + self.b2)
        return y
    
    def train_step(self, x, y_true, learning_rate=0.001):
        """Single training step using gradient descent."""
        # Forward pass
        h = np.maximum(0, np.dot(x, self.w1) + self.b1)
        y_pred = np.tanh(np.dot(h, self.w2) + self.b2)
        
        # Compute loss (MSE)
        loss = np.mean((y_pred - y_true) ** 2)
        
        # Backward pass (simplified)
        # This is a basic implementation - use proper frameworks for real training
        dy = 2 * (y_pred - y_true) / len(y_true)
        dy_tanh = dy * (1 - y_pred ** 2)
        
        dw2 = np.dot(h.T, dy_tanh)
        db2 = np.sum(dy_tanh, axis=0)
        
        dh = np.dot(dy_tanh, self.w2.T)
        dh_relu = dh * (h > 0)
        
        dw1 = np.dot(x.T, dh_relu)
        db1 = np.sum(dh_relu, axis=0)
        
        # Update weights
        self.w1 -= learning_rate * dw1
        self.b1 -= learning_rate * db1
        self.w2 -= learning_rate * dw2
        self.b2 -= learning_rate * db2
        
        return loss

def analyze_human_control_patterns(dataset: TrainingDataset):
    """Analyze patterns in human control behavior."""
    print("\n=== Human Control Pattern Analysis ===")
    
    # Convert to DataFrame for easier analysis
    df = pd.DataFrame(dataset.states, columns=dataset.feature_names)
    df['rudder'] = dataset.actions[:, 0]
    df['sail'] = dataset.actions[:, 1]
    df['session'] = dataset.session_ids
    
    print(f"Dataset shape: {df.shape}")
    print(f"Feature columns: {list(df.columns)}")
    
    # Basic statistics
    print("\nControl action statistics:")
    print(f"Rudder: mean={df['rudder'].mean():.3f}, std={df['rudder'].std():.3f}, range=[{df['rudder'].min():.3f}, {df['rudder'].max():.3f}]")
    print(f"Sail: mean={df['sail'].mean():.3f}, std={df['sail'].std():.3f}, range=[{df['sail'].min():.3f}, {df['sail'].max():.3f}]")
    
    # Analyze heading error vs rudder response
    if 'heading_error' in df.columns:
        print(f"\nHeading error statistics:")
        print(f"Mean: {df['heading_error'].mean():.1f}°, Std: {df['heading_error'].std():.1f}°")
        
        # Correlation between heading error and rudder command
        correlation = df['heading_error'].corr(df['rudder'])
        print(f"Heading error vs rudder correlation: {correlation:.3f}")
    
    # Session-wise analysis
    print(f"\nSession analysis:")
    session_stats = df.groupby('session').agg({
        'rudder': ['mean', 'std'],
        'sail': ['mean', 'std'],
        'heading_error': ['mean', 'std'] if 'heading_error' in df.columns else ['count']
    }).round(3)
    print(session_stats)

def train_neural_controller_example(dataset: TrainingDataset):
    """Example of training a neural controller on human data."""
    print("\n=== Training Neural Controller ===")
    
    # Split data
    train_dataset, val_dataset = dataset.split(train_ratio=0.8)
    print(f"Training samples: {len(train_dataset)}, Validation samples: {len(val_dataset)}")
    
    # Normalize features (important for neural networks)
    mean = np.mean(train_dataset.states, axis=0)
    std = np.std(train_dataset.states, axis=0) + 1e-8  # Avoid division by zero
    
    train_states_norm = (train_dataset.states - mean) / std
    val_states_norm = (val_dataset.states - mean) / std
    
    # Initialize controller
    controller = SimpleNeuralController(train_states_norm.shape[1])
    
    # Training loop
    epochs = 100
    batch_size = 32
    
    train_losses = []
    val_losses = []
    
    for epoch in range(epochs):
        # Shuffle training data
        indices = np.random.permutation(len(train_dataset))
        epoch_losses = []
        
        # Mini-batch training
        for i in range(0, len(train_dataset), batch_size):
            batch_idx = indices[i:i+batch_size]
            batch_x = train_states_norm[batch_idx]
            batch_y = train_dataset.actions[batch_idx]
            
            loss = controller.train_step(batch_x, batch_y)
            epoch_losses.append(loss)
        
        train_loss = np.mean(epoch_losses)
        train_losses.append(train_loss)
        
        # Validation loss
        val_pred = controller.forward(val_states_norm)
        val_loss = np.mean((val_pred - val_dataset.actions) ** 2)
        val_losses.append(val_loss)
        
        if epoch % 20 == 0:
            print(f"Epoch {epoch}: Train Loss = {train_loss:.4f}, Val Loss = {val_loss:.4f}")
    
    print(f"Final: Train Loss = {train_losses[-1]:.4f}, Val Loss = {val_losses[-1]:.4f}")
    
    # Test controller on a few samples
    print("\n=== Controller Test ===")
    test_indices = np.random.choice(len(val_dataset), 5)
    for i in test_indices:
        state = val_states_norm[i:i+1]
        human_action = val_dataset.actions[i]
        neural_action = controller.forward(state)[0]
        
        print(f"Sample {i}:")
        print(f"  Human:  rudder={human_action[0]:+.3f}, sail={human_action[1]:+.3f}")
        print(f"  Neural: rudder={neural_action[0]:+.3f}, sail={neural_action[1]:+.3f}")
        print(f"  Error:  rudder={abs(human_action[0] - neural_action[0]):.3f}, sail={abs(human_action[1] - neural_action[1]):.3f}")

def plot_training_data(dataset: TrainingDataset, output_dir: str = "plots"):
    """Create visualizations of the training data."""
    print(f"\n=== Creating plots in {output_dir} ===")
    
    Path(output_dir).mkdir(exist_ok=True)
    
    # Plot 1: Control actions over time
    plt.figure(figsize=(12, 6))
    
    plt.subplot(2, 1, 1)
    plt.plot(dataset.timestamps, dataset.actions[:, 0], 'b-', alpha=0.7, label='Rudder')
    plt.ylabel('Rudder Command')
    plt.legend()
    plt.grid(True)
    
    plt.subplot(2, 1, 2)
    plt.plot(dataset.timestamps, dataset.actions[:, 1], 'r-', alpha=0.7, label='Sail')
    plt.xlabel('Time (s)')
    plt.ylabel('Sail Command')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.savefig(f"{output_dir}/control_actions_time.png", dpi=150)
    plt.close()
    
    # Plot 2: Heading error vs rudder command
    if 'heading_error' in dataset.feature_names:
        heading_error_idx = dataset.feature_names.index('heading_error')
        heading_errors = dataset.states[:, heading_error_idx]
        
        plt.figure(figsize=(10, 6))
        plt.scatter(heading_errors, dataset.actions[:, 0], alpha=0.5, s=1)
        plt.xlabel('Heading Error (degrees)')
        plt.ylabel('Rudder Command')
        plt.title('Human Control Response: Heading Error vs Rudder')
        plt.grid(True)
        
        # Add trend line
        z = np.polyfit(heading_errors, dataset.actions[:, 0], 1)
        p = np.poly1d(z)
        x_line = np.linspace(heading_errors.min(), heading_errors.max(), 100)
        plt.plot(x_line, p(x_line), "r--", alpha=0.8, label=f'Trend: y={z[0]:.3f}x+{z[1]:.3f}')
        plt.legend()
        
        plt.tight_layout()
        plt.savefig(f"{output_dir}/heading_error_vs_rudder.png", dpi=150)
        plt.close()
    
    print(f"Plots saved to {output_dir}/")

def main():
    parser = argparse.ArgumentParser(description='Training data analysis and neural controller example')
    parser.add_argument('--data_dir', default='training_data', help='Directory containing training data')
    parser.add_argument('--output_dir', default='analysis_output', help='Output directory for plots and results')
    parser.add_argument('--skip_training', action='store_true', help='Skip neural network training')
    args = parser.parse_args()
    
    try:
        # Load training data
        loader = TrainingDataLoader(args.data_dir)
        dataset = loader.load_all_sessions()
        
        # Analyze human control patterns
        analyze_human_control_patterns(dataset)
        
        # Create visualizations
        plot_training_data(dataset, args.output_dir)
        
        # Train neural controller example
        if not args.skip_training and len(dataset) > 100:  # Need enough data
            train_neural_controller_example(dataset)
        else:
            print(f"\nSkipping neural training (need >100 samples, have {len(dataset)})")
        
        print(f"\n=== Analysis complete! ===")
        print(f"Check {args.output_dir}/ for plots and results")
        
    except Exception as e:
        print(f"Error: {e}")
        print("\nTo collect training data:")
        print("1. Set 'data_collection_enabled: true' in argo_refactored.yaml")
        print("2. Run the refactored control node")
        print("3. Operate the boat in human control mode")
        print("4. Training data will be saved to training_data/ directory")

if __name__ == '__main__':
    main()
