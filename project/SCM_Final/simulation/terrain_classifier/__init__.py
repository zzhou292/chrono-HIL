"""
Terrain Classification Module
==============================

Classifies terrain type (clay / sand / dirt) from vehicle-measurable signals:
  - Wheel speeds → slip ratio statistics
  - IMU → longitudinal/lateral acceleration, yaw rate, vibration
  - GPS → position, heading, speed

Architecture:
  1. FeatureExtractor       — sliding-window feature computation from VehicleState
  2. collect_data.py        — subscribe to sim, record features + ground truth labels
  3. train_model.py         — train a HistGradientBoosting classifier on collected data
  4. classifier_node.py     — ZMQ node: subscribe to state, publish TerrainEstimate
  5. launch_data_collection — automated multi-run data collection orchestrator

See README.md for end-to-end usage instructions.
"""

from terrain_classifier.feature_extractor import FeatureExtractor

__all__ = ["FeatureExtractor"]
