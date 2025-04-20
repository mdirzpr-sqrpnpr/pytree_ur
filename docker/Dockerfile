# Base image
FROM python:3.12-slim

# Set working directory
WORKDIR /app

# Copy code into container
COPY . /app

# Install system dependencies
RUN apt-get update && apt-get install -y graphviz && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Start container in shell so user can run commands manually
CMD ["/bin/bash"]
