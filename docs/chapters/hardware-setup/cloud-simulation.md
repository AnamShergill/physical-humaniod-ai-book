---
sidebar_label: 'Cloud Simulation Setup'
---

# Cloud Simulation Setup for Robotics Development

## Overview

Cloud-based simulation platforms provide access to powerful computational resources for robotics development without requiring local high-end hardware. This guide covers setting up and using cloud simulation services, particularly NVIDIA Omniverse Cloud and Isaac Sim Cloud, for robotics applications and AI training.

## Learning Objectives

By the end of this lesson, you will be able to:
- Set up cloud simulation environments for robotics
- Configure remote access to simulation platforms
- Optimize cloud simulation for different use cases
- Manage costs and resources effectively
- Integrate cloud simulation with local development workflows

## Cloud Simulation Platforms Overview

### NVIDIA Omniverse Cloud

NVIDIA Omniverse Cloud provides professional-grade simulation capabilities:

```yaml
Features:
  - GPU-accelerated rendering and physics
  - Multi-user collaboration
  - Real-time USD scene synchronization
  - Scalable compute resources
  - Integration with Isaac Sim

Use Cases:
  - Large-scale environment simulation
  - Multi-robot scenarios
  - AI training at scale
  - Collaborative development
  - High-fidelity rendering
```

### Platform Comparison

```python
# Comparison of cloud simulation platforms

def compare_cloud_platforms():
    platforms = {
        'NVIDIA Omniverse Cloud': {
            'strengths': ['High-fidelity graphics', 'Physics accuracy', 'ROS integration'],
            'cost': 'Enterprise pricing',
            'best_for': 'Professional robotics development'
        },
        'AWS RoboMaker Simulation': {
            'strengths': ['Native AWS integration', 'Scalable', 'Cost-effective'],
            'cost': 'Pay-per-simulation',
            'best_for': 'ROS-based applications'
        },
        'Google Cloud Robotics': {
            'strengths': ['AI/ML integration', 'Kubernetes support', 'Big data'],
            'cost': 'Compute + storage fees',
            'best_for': 'AI-driven robotics'
        },
        'Azure Digital Twins': {
            'strengths': ['IoT integration', 'Digital twin modeling', 'Enterprise'],
            'cost': 'Resource-based',
            'best_for': 'Industrial IoT robotics'
        }
    }

    return platforms

platforms = compare_cloud_platforms()
for name, info in platforms.items():
    print(f"{name}: {info['best_for']} - {info['cost']}")
```

## NVIDIA Omniverse Cloud Setup

### Account and Access Setup

```bash
# Sign up for NVIDIA Developer account
# Access Omniverse Cloud through NVIDIA Developer portal

# Install Omniverse Launcher
# Download from developer.nvidia.com

# Configure cloud connection
cat > ~/.nvidia-omniverse/config.json << 'EOF'
{
  "cloud": {
    "enabled": true,
    "server_url": "https://omniverse.nvidia.com",
    "api_key": "YOUR_API_KEY",
    "workspace": "robotics-lab"
  },
  "local": {
    "cache_dir": "/home/user/.nvidia-omniverse/cache",
    "temp_dir": "/tmp/omniverse"
  }
}
EOF
```

### Omniverse Connect Configuration

```python
# Python script to connect to Omniverse Cloud
import omni
from omni.kit.viewport.window import get_viewport_window
from pxr import Usd, Sdf, UsdGeom

class OmniverseCloudConnector:
    def __init__(self, server_url, api_key):
        self.server_url = server_url
        self.api_key = api_key
        self.stage = None

    def connect_to_cloud(self):
        """Connect to Omniverse Cloud server"""
        try:
            # Initialize connection
            from omni.kit.core.extension import get_extension_from_path
            from omniverse import carb

            # Set connection parameters
            carb.settings.get_settings().set("/exts/omni.services.client.live.server/url", self.server_url)
            carb.settings.get_settings().set("/exts/omni.services.client.live.server/api_key", self.api_key)

            print(f"Connected to Omniverse Cloud at {self.server_url}")
            return True
        except Exception as e:
            print(f"Failed to connect to Omniverse Cloud: {e}")
            return False

    def open_cloud_stage(self, stage_path):
        """Open a stage from cloud storage"""
        try:
            # Open USD stage from cloud
            self.stage = omni.usd.get_context().open_stage(stage_path)
            print(f"Opened stage: {stage_path}")
            return self.stage
        except Exception as e:
            print(f"Failed to open stage {stage_path}: {e}")
            return None

    def create_robot_scenario(self, robot_model_path, environment_path):
        """Create a robot simulation scenario"""
        if not self.stage:
            print("No stage opened. Connect first.")
            return

        stage = self.stage

        # Add robot to scene
        robot_prim = stage.DefinePrim("/World/Robot", "Xform")
        robot_prim.GetReferences().AddReference(robot_model_path)

        # Add environment
        env_prim = stage.DefinePrim("/World/Environment", "Xform")
        env_prim.GetReferences().AddReference(environment_path)

        print("Robot scenario created in cloud simulation")
        return robot_prim, env_prim

# Example usage
connector = OmniverseCloudConnector(
    server_url="https://omniverse.nvidia.com",
    api_key="your-api-key"
)

if connector.connect_to_cloud():
    stage = connector.open_cloud_stage("omniverse://server.example.com/Projects/RoboticsLab/Scenario1.usd")
    if stage:
        connector.create_robot_scenario(
            robot_model_path="omniverse://server.example.com/Assets/Robots/TurtleBot3.usd",
            environment_path="omniverse://server.example.com/Assets/Environments/Office.usd"
        )
```

## Isaac Sim Cloud Configuration

### Cloud Instance Setup

```bash
# Isaac Sim Cloud setup via command line
# Using NVIDIA Cloud Launch or Kubernetes

# Example Kubernetes deployment for Isaac Sim Cloud
cat > isaac-sim-cloud.yaml << 'EOF'
apiVersion: v1
kind: Service
metadata:
  name: isaac-sim-service
spec:
  selector:
    app: isaac-sim
  ports:
    - protocol: TCP
      port: 8211  # Omniverse streaming
      targetPort: 8211
    - protocol: TCP
      port: 5557  # ROS bridge
      targetPort: 5557
  type: LoadBalancer

---
apiVersion: apps/v1
kind: Deployment
metadata:
  name: isaac-sim-deployment
spec:
  replicas: 1
  selector:
    matchLabels:
      app: isaac-sim
  template:
    metadata:
      labels:
        app: isaac-sim
    spec:
      containers:
      - name: isaac-sim
        image: nvcr.io/nvidia/isaac-sim:latest
        resources:
          requests:
            nvidia.com/gpu: 1
            memory: "16Gi"
            cpu: "8"
          limits:
            nvidia.com/gpu: 1
            memory: "32Gi"
            cpu: "16"
        env:
        - name: "OMNIVERSE_HEADLESS"
          value: "1"
        - name: "ROS_DOMAIN_ID"
          value: "1"
        ports:
        - containerPort: 8211
        - containerPort: 5557
        volumeMounts:
        - name: workspace-volume
          mountPath: /workspace
      volumes:
      - name: workspace-volume
        persistentVolumeClaim:
          claimName: isaac-sim-pvc
EOF
```

### Remote Access Configuration

```bash
# SSH tunneling for secure access
# Set up SSH tunnel to cloud instance
ssh -L 8211:localhost:8211 -L 5557:localhost:5557 user@cloud-instance-ip

# Or use VPN for secure connection
# Configure VPN client for cloud network access

# Test connection
telnet localhost 8211  # Should connect to Omniverse streaming
telnet localhost 5557  # Should connect to ROS bridge
```

## Cloud Simulation Workflows

### Development Workflow Integration

```python
# Cloud simulation integration with local development
import os
import subprocess
import paramiko
from pathlib import Path

class CloudSimulationManager:
    def __init__(self, cloud_host, ssh_key_path):
        self.cloud_host = cloud_host
        self.ssh_key_path = ssh_key_path
        self.ssh_client = None

    def connect_to_cloud(self):
        """Establish SSH connection to cloud instance"""
        try:
            self.ssh_client = paramiko.SSHClient()
            self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            self.ssh_client.connect(
                hostname=self.cloud_host,
                username='ubuntu',
                key_filename=self.ssh_key_path
            )
            print(f"Connected to cloud instance: {self.cloud_host}")
            return True
        except Exception as e:
            print(f"Failed to connect to cloud: {e}")
            return False

    def sync_local_to_cloud(self, local_path, remote_path):
        """Sync local files to cloud instance"""
        try:
            # Use SCP to sync files
            sftp = self.ssh_client.open_sftp()
            local_path = Path(local_path)

            if local_path.is_file():
                sftp.put(str(local_path), remote_path)
            else:
                # For directories, create remote directory and sync contents
                self._sync_directory(sftp, local_path, remote_path)

            print(f"Synced {local_path} to {remote_path}")
            return True
        except Exception as e:
            print(f"Sync failed: {e}")
            return False

    def _sync_directory(self, sftp, local_dir, remote_dir):
        """Recursively sync directory contents"""
        local_dir = Path(local_dir)
        sftp.mkdir(remote_dir, ignore_existing=True)

        for item in local_dir.iterdir():
            remote_item_path = f"{remote_dir}/{item.name}"
            if item.is_file():
                sftp.put(str(item), remote_item_path)
            elif item.is_dir():
                self._sync_directory(sftp, item, remote_item_path)

    def launch_simulation(self, scenario_file, gpu_config="high"):
        """Launch simulation on cloud instance"""
        try:
            # Prepare launch command
            cmd = f"""
            cd /opt/isaac-sim && \\
            export ISAACSIM_HEADLESS=1 && \\
            export CUDA_VISIBLE_DEVICES=0 && \\
            python -c "
            from omni.isaac.kit import SimulationApp
            simulation_app = SimulationApp(launch_args_dict={{"headless": True}})
            from omni.isaac.core import World
            from omni.isaac.core.robots import Robot
            import carb

            # Load scenario
            world = World(stage_units_in_meters=1.0)
            # Add your scenario loading code here
            world.reset()

            # Run simulation for specified time
            for i in range(1000):  # Run for 1000 steps
                world.step(render=False)

            simulation_app.close()
            "
            """

            stdin, stdout, stderr = self.ssh_client.exec_command(cmd)

            # Monitor output
            for line in stdout:
                print(line.strip())

            for line in stderr:
                print(f"ERROR: {line.strip()}")

            return True
        except Exception as e:
            print(f"Simulation launch failed: {e}")
            return False

    def get_simulation_results(self, remote_results_path, local_results_path):
        """Download simulation results from cloud"""
        try:
            sftp = self.ssh_client.open_sftp()
            sftp.get(remote_results_path, local_results_path)
            print(f"Downloaded results to {local_results_path}")
            return True
        except Exception as e:
            print(f"Failed to download results: {e}")
            return False

# Example usage
cloud_manager = CloudSimulationManager(
    cloud_host="isaac-sim-cloud.example.com",
    ssh_key_path="~/.ssh/cloud_key.pem"
)

if cloud_manager.connect_to_cloud():
    # Sync local scenario to cloud
    cloud_manager.sync_local_to_cloud(
        "local_scenarios/navigation_test.py",
        "/workspace/scenarios/navigation_test.py"
    )

    # Launch simulation
    cloud_manager.launch_simulation("navigation_test.py")

    # Download results
    cloud_manager.get_simulation_results(
        "/workspace/results/simulation_output.json",
        "local_results/simulation_output.json"
    )
```

### Cost Optimization Strategies

```python
# Cost optimization for cloud simulation
import boto3  # For AWS, similar for other clouds
from datetime import datetime, timedelta

class CloudCostOptimizer:
    def __init__(self):
        self.billing_client = boto3.client('ce')  # Cost Explorer
        self.ec2_client = boto3.client('ec2')

    def get_simulation_cost_estimate(self, instance_type, hours, region='us-west-2'):
        """Estimate cost for simulation run"""
        pricing_client = boto3.client('pricing', region_name='us-east-1')

        response = pricing_client.get_products(
            ServiceCode='AmazonEC2',
            Filters=[
                {'Type': 'TERM_MATCH', 'Field': 'ServiceCode', 'Value': 'AmazonEC2'},
                {'Type': 'TERM_MATCH', 'Field': 'instanceType', 'Value': instance_type},
                {'Type': 'TERM_MATCH', 'Field': 'location', 'Value': region},
                {'Type': 'TERM_MATCH', 'Field': 'productFamily', 'Value': 'Compute Instance'},
            ]
        )

        # Parse pricing data and calculate cost
        # Implementation would involve parsing AWS pricing API response
        estimated_cost = 0.0  # Placeholder
        return estimated_cost

    def optimize_instance_selection(self, simulation_requirements):
        """Select optimal instance based on requirements"""
        requirements = {
            'gpu_needed': True,
            'memory_gb': 32,
            'cpu_cores': 8,
            'duration_hours': 10,
            'priority': 'cost'  # or 'performance'
        }

        # Instance options
        instances = [
            {'type': 'g4dn.xlarge', 'vcpu': 4, 'memory': 16, 'gpu': 1, 'cost': 0.5},
            {'type': 'g4dn.2xlarge', 'vcpu': 8, 'memory': 32, 'gpu': 1, 'cost': 0.75},
            {'type': 'g4dn.4xlarge', 'vcpu': 16, 'memory': 64, 'gpu': 1, 'cost': 1.2},
            {'type': 'p3.2xlarge', 'vcpu': 8, 'memory': 61, 'gpu': 1, 'cost': 3.06}
        ]

        # Filter by requirements
        suitable_instances = [
            inst for inst in instances
            if (inst['vcpu'] >= requirements['cpu_cores'] and
                inst['memory'] >= requirements['memory_gb'])
        ]

        if requirements['priority'] == 'cost':
            optimal = min(suitable_instances, key=lambda x: x['cost'])
        else:  # performance
            optimal = max(suitable_instances, key=lambda x: x['gpu'])

        return optimal

    def schedule_off_peak_runs(self):
        """Schedule simulation runs during off-peak hours for cost savings"""
        # Check spot instance availability for cost savings
        ec2_client = boto3.client('ec2')

        response = ec2_client.describe_spot_price_history(
            InstanceTypes=['g4dn.2xlarge'],
            StartTime=datetime.now() - timedelta(hours=1),
            EndTime=datetime.now()
        )

        # Find best spot price
        if response['SpotPriceHistory']:
            best_spot = min(response['SpotPriceHistory'], key=lambda x: float(x['SpotPrice']))
            print(f"Best spot price: {best_spot['SpotPrice']} for {best_spot['InstanceType']}")
            return best_spot
        return None

# Example usage
optimizer = CloudCostOptimizer()

# Get cost estimate
cost = optimizer.get_simulation_cost_estimate('g4dn.2xlarge', 10)
print(f"Estimated cost: ${cost}")

# Get optimal instance
optimal = optimizer.optimize_instance_selection({})
print(f"Optimal instance: {optimal}")

# Check spot pricing
spot_info = optimizer.schedule_off_peak_runs()
if spot_info:
    print(f"Spot pricing available: {spot_info['SpotPrice']}")
```

## Performance Optimization

### Cloud-Specific Optimizations

```bash
# Optimizations for cloud simulation performance

# Network optimization for streaming
# Set up VPC peering or dedicated connections
# Use cloud regions closest to your location

# Instance optimization
cat > cloud_simulation_config.sh << 'EOF'
#!/bin/bash

# GPU optimization
nvidia-smi -pm 1  # Enable persistence mode
nvidia-smi -ac 5001,1590  # Set max GPU clock (adjust for your GPU)

# Memory optimization
echo 'vm.swappiness=1' | sudo tee -a /etc/sysctl.conf
echo 'vm.dirty_ratio=15' | sudo tee -a /etc/sysctl.conf
echo 'vm.dirty_background_ratio=5' | sudo tee -a /etc/sysctl.conf

# Network optimization for streaming
echo 'net.core.rmem_max = 134217728' | sudo tee -a /etc/sysctl.conf
echo 'net.core.wmem_max = 134217728' | sudo tee -a /etc/sysctl.conf

# Apply settings
sudo sysctl -p

# Install monitoring tools
sudo apt install -y htop nethogs iftop

echo "Cloud simulation optimizations applied"
EOF

chmod +x cloud_simulation_config.sh
```

### Parallel Simulation Management

```python
# Managing multiple parallel simulations in cloud
import concurrent.futures
import threading
from typing import List, Dict

class ParallelSimulationManager:
    def __init__(self, max_parallel=4):
        self.max_parallel = max_parallel
        self.running_simulations = {}
        self.simulation_lock = threading.Lock()

    def run_batch_simulations(self, simulation_configs: List[Dict]):
        """Run multiple simulations in parallel"""
        results = []

        with concurrent.futures.ThreadPoolExecutor(max_workers=self.max_parallel) as executor:
            # Submit all simulation jobs
            future_to_config = {
                executor.submit(self.run_single_simulation, config): config
                for config in simulation_configs
            }

            # Collect results as they complete
            for future in concurrent.futures.as_completed(future_to_config):
                config = future_to_config[future]
                try:
                    result = future.result()
                    results.append({
                        'config': config,
                        'result': result,
                        'status': 'completed'
                    })
                    print(f"Completed simulation: {config['name']}")
                except Exception as e:
                    results.append({
                        'config': config,
                        'result': None,
                        'status': 'failed',
                        'error': str(e)
                    })
                    print(f"Failed simulation {config['name']}: {e}")

        return results

    def run_single_simulation(self, config):
        """Run a single simulation with given config"""
        # This would connect to cloud instance and run simulation
        # Implementation depends on specific cloud platform

        simulation_name = config.get('name', 'unnamed')
        duration = config.get('duration', 60)  # seconds
        scenario = config.get('scenario', 'default')

        print(f"Starting simulation: {simulation_name}")

        # Simulate running (in real implementation, this would call cloud API)
        import time
        time.sleep(duration)  # Simulate simulation time

        # Return simulation results
        return {
            'name': simulation_name,
            'duration': duration,
            'scenario': scenario,
            'success': True,
            'metrics': {
                'avg_fps': 60,
                'memory_usage': '8GB',
                'gpu_utilization': '85%'
            }
        }

# Example usage
manager = ParallelSimulationManager(max_parallel=3)

simulation_configs = [
    {'name': 'nav_test_1', 'duration': 30, 'scenario': 'maze_navigation'},
    {'name': 'nav_test_2', 'duration': 45, 'scenario': 'obstacle_avoidance'},
    {'name': 'nav_test_3', 'duration': 60, 'scenario': 'multi_floor_navigation'},
    {'name': 'nav_test_4', 'duration': 35, 'scenario': 'dynamic_obstacles'}
]

results = manager.run_batch_simulations(simulation_configs)

for result in results:
    print(f"Simulation {result['config']['name']}: {result['status']}")
```

## Security and Access Management

### Secure Access Configuration

```yaml
# Security configuration for cloud simulation
security:
  authentication:
    - multi_factor_auth: true
    - api_key_rotation: 90_days
    - ip_whitelisting: enabled

  encryption:
    - data_at_rest: aes_256
    - data_in_transit: tls_1_3
    - gpu_memory: encrypted

  network:
    - vpc_configuration: required
    - private_subnet: true
    - vpn_access: required
    - firewall_rules:
        - allow: [5557, 8211, 22]  # ROS, Omniverse, SSH
        - deny: [0-65535]  # All others

  monitoring:
    - audit_logs: enabled
    - intrusion_detection: enabled
    - gpu_usage_monitoring: enabled
```

### Access Control Implementation

```python
# Access control for cloud simulation resources
from functools import wraps
import jwt
from datetime import datetime, timedelta

class CloudAccessManager:
    def __init__(self, secret_key):
        self.secret_key = secret_key
        self.active_tokens = set()

    def generate_access_token(self, user_id, permissions, expiry_hours=24):
        """Generate JWT token for cloud access"""
        payload = {
            'user_id': user_id,
            'permissions': permissions,
            'exp': datetime.utcnow() + timedelta(hours=expiry_hours),
            'iat': datetime.utcnow()
        }

        token = jwt.encode(payload, self.secret_key, algorithm='HS256')
        self.active_tokens.add(token)
        return token

    def validate_token(self, token):
        """Validate access token"""
        try:
            if token not in self.active_tokens:
                return False

            payload = jwt.decode(token, self.secret_key, algorithms=['HS256'])
            return payload
        except jwt.ExpiredSignatureError:
            self.active_tokens.discard(token)
            return False
        except jwt.InvalidTokenError:
            return False

    def require_access(self, required_permissions):
        """Decorator to require specific permissions"""
        def decorator(func):
            @wraps(func)
            def wrapper(*args, **kwargs):
                # Extract token from request (simplified)
                token = kwargs.get('token') or args[0].get('token')

                if not token:
                    raise PermissionError("No access token provided")

                user_data = self.validate_token(token)
                if not user_data:
                    raise PermissionError("Invalid or expired token")

                user_permissions = user_data.get('permissions', [])
                if not all(perm in user_permissions for perm in required_permissions):
                    raise PermissionError(f"Insufficient permissions. Required: {required_permissions}")

                return func(*args, **kwargs)
            return wrapper
        return decorator

# Example usage
access_manager = CloudAccessManager("your-secret-key")

# Generate token for user
token = access_manager.generate_access_token(
    user_id="user123",
    permissions=["read", "write", "execute_simulation"]
)

@access_manager.require_access(["execute_simulation"])
def run_simulation_scenario(scenario_data, token=None):
    """Run simulation with access control"""
    print(f"Running simulation with scenario: {scenario_data}")
    return {"status": "success", "simulation_id": "sim_123"}

# This would work
result = run_simulation_scenario({"type": "navigation"}, token=token)
print(result)
```

## Troubleshooting and Monitoring

### Cloud Simulation Monitoring

```bash
# Monitoring script for cloud simulation instances
cat > cloud_monitor.sh << 'EOF'
#!/bin/bash

# Monitor cloud simulation performance
INSTANCE_ID="i-1234567890abcdef0"
REGION="us-west-2"

echo "Monitoring Cloud Simulation Instance: $INSTANCE_ID"

while true; do
    echo "=== $(date) ==="

    # Get cloudwatch metrics
    aws cloudwatch get-metric-statistics \
        --namespace AWS/EC2 \
        --metric-name CPUUtilization \
        --dimensions Name=InstanceId,Value=$INSTANCE_ID \
        --start-time $(date -u -d '5 minutes ago' +%Y-%m-%dT%H:%M:%S) \
        --end-time $(date -u +%Y-%m-%dT%H:%M:%S) \
        --period 300 \
        --statistics Average \
        --region $REGION

    # Check GPU utilization (if available)
    if command -v nvidia-smi &> /dev/null; then
        echo "GPU Utilization:"
        nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total --format=csv
    fi

    # Check network usage
    echo "Network usage:"
    cat /proc/net/dev | grep eth0

    # Check disk space
    echo "Disk usage:"
    df -h | grep -E 'overlay|/dev'

    sleep 60
done
EOF

chmod +x cloud_monitor.sh
```

### Common Cloud Issues and Solutions

```python
# Troubleshooting cloud simulation issues
class CloudSimulationTroubleshooter:
    def __init__(self):
        self.known_issues = {
            'connection_timeout': {
                'symptoms': ['Cannot connect to simulation', 'Timeout errors'],
                'causes': ['Network issues', 'Firewall blocking', 'Instance not ready'],
                'solutions': [
                    'Check VPN connection',
                    'Verify firewall rules',
                    'Ensure instance is running'
                ]
            },
            'gpu_not_detected': {
                'symptoms': ['CUDA error', 'GPU not found'],
                'causes': ['Driver issues', 'Wrong instance type'],
                'solutions': [
                    'Verify GPU instance type',
                    'Check NVIDIA drivers',
                    'Install CUDA toolkit'
                ]
            },
            'performance_degradation': {
                'symptoms': ['Low FPS', 'High latency', 'Slow response'],
                'causes': ['Resource contention', 'Network bandwidth'],
                'solutions': [
                    'Scale up instance',
                    'Check network connection',
                    'Optimize simulation settings'
                ]
            }
        }

    def diagnose_issue(self, symptoms):
        """Diagnose issue based on symptoms"""
        matching_issues = []
        symptoms_lower = [s.lower() for s in symptoms]

        for issue_name, issue_data in self.known_issues.items():
            issue_symptoms = [s.lower() for s in issue_data['symptoms']]
            if any(symptom in issue_symptoms for symptom in symptoms_lower):
                matching_issues.append({
                    'issue': issue_name,
                    'data': issue_data
                })

        return matching_issues

    def get_troubleshooting_guide(self, issue_name):
        """Get detailed troubleshooting guide for specific issue"""
        if issue_name in self.known_issues:
            issue = self.known_issues[issue_name]
            guide = f"""
            Issue: {issue_name}
            Symptoms: {', '.join(issue['symptoms'])}
            Possible Causes: {', '.join(issue['causes'])}
            Solutions:
            """
            for i, solution in enumerate(issue['solutions'], 1):
                guide += f"  {i}. {solution}\n"
            return guide
        return f"Issue '{issue_name}' not found in known issues."

# Example usage
troubleshooter = CloudSimulationTroubleshooter()

# Diagnose based on symptoms
symptoms = ["Cannot connect to simulation", "Timeout errors"]
issues = troubleshooter.diagnose_issue(symptoms)

for issue in issues:
    print(troubleshooter.get_troubleshooting_guide(issue['issue']))
```

## Practical Exercise

Set up and test a complete cloud simulation environment:

1. Configure access to a cloud simulation platform
2. Launch a simulation instance
3. Run a simple robot navigation scenario
4. Monitor performance and costs
5. Implement access control and security measures
6. Optimize for your specific use case

## Summary

Cloud simulation platforms provide scalable, high-performance resources for robotics development without requiring local high-end hardware. Proper configuration, security, and optimization enable efficient development and testing of robotics applications in professional environments.

## Next Steps

Complete the hardware and lab setup chapter by reviewing best practices for integrating cloud and local simulation environments, managing workflows across different platforms, and planning for production deployment of robotics applications.