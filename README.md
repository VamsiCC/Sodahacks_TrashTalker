<div align="center">
  <h1>🗑️ TrashTalker</h1>
  <p><strong>AI-Powered Waste Classification for Sustainable Living</strong></p>
  
  [![Sodahacks 2025](https://img.shields.io/badge/Sodahacks_2025-Winner-brightgreen?style=flat-square)](https://sodahacks.com)
  [![License](https://img.shields.io/badge/License-MIT-blue.svg?style=flat-square)]()
  [![Language](https://img.shields.io/badge/Language-C%20%26%20Python-orange?style=flat-square)]()
</div>

---

## 🌍 Overview

**TrashTalker** is the **Sodahacks 2025 Sustainability Track Winner** – an intelligent waste classification system that leverages **AI and Computer Vision** to automatically identify and categorize waste materials. By simplifying the waste sorting process, TrashTalker empowers individuals and organizations to make better environmental choices and reduce contamination in recycling streams.

Whether you're sorting household waste or managing industrial materials, TrashTalker provides real-time, accurate classifications to streamline sustainability efforts.

---

## ✨ Key Features

- **📷 Real-Time Computer Vision**: Uses ESP32-CAM for instant waste image capture and analysis
- **🧠 AI-Powered Classification**: Integrates OpenAI's vision models via OpenRouter API for accurate waste categorization
- **🌐 Web Interface**: User-friendly web UI for triggering classifications and viewing results
- **⚡ Hardware Integration**: Built on ESP32-CAM and ESP32-S3 for edge computing efficiency
- **🔄 Cloud Bridge**: Seamless AWS S3 integration for image storage and analysis
- **📊 Manual & Automatic Modes**: Supports both on-demand and automated classification workflows

---

## 🏆 Achievements

- **Winner** 🥇 Sodahacks 2025 Sustainability Track
- Demonstrated innovative approach to waste management and environmental impact
- Practical solution for real-world sustainability challenges

---

## 🛠️ Tech Stack

| Component | Technology |
|-----------|-----------|
| **Microcontroller** | ESP32-CAM, ESP32-S3 |
| **Firmware** | C (ESP-IDF Framework) |
| **Backend** | Python |
| **AI/ML** | OpenAI Vision Models (via OpenRouter API) |
| **Image Processing** | JPEG compression, YUV conversion |
| **Cloud Storage** | AWS S3 |
| **Web Interface** | HTTP Server (ESP-IDF) |

---

## 📋 Project Structure

```
Sodahacks_TrashTalker/
├── camera_test/               # Main ESP32-CAM firmware
│   ├── main/
│   │   └── main.c            # WiFi, HTTP server, classification logic
│   └── managed_components/   # ESP32 libraries (camera, JPEG, etc.)
├── bridge_to_s3.py           # Python script for AWS S3 integration
└── sdkconfig.defaults        # ESP-IDF configuration
```

---

## 🚀 Quick Start

### Prerequisites
- ESP32-CAM development board
- ESP32-S3 toolkit (optional, for enhanced processing)
- Python 3.8+
- AWS S3 credentials (for cloud integration)
- OpenRouter API key (for AI classification)

### Hardware Setup

1. **ESP32-CAM Configuration**
   ```c
   // Update WiFi credentials in camera_test/main/main.c
   #define WIFI_STA_SSID       "Your_WiFi_SSID"
   #define WIFI_STA_PASS       "Your_WiFi_Password"
   
   // Configure OpenRouter API
   #define OPENROUTER_API_KEY  "your-api-key-here"
   ```

2. **Build and Flash**
   ```bash
   cd camera_test
   idf.py build
   idf.py -p /dev/ttyUSB0 flash monitor
   ```

### Software Setup

3. **Install Python Dependencies**
   ```bash
   pip install boto3 python-dotenv
   ```

4. **Configure AWS S3 Bridge**
   ```python
   # Update credentials in bridge_to_s3.py
   AWS_BUCKET_NAME = "your-bucket-name"
   AWS_REGION = "us-east-1"
   ```

5. **Run Classification**
   - **Via Web UI**: Access `http://<esp32-ip>` and click "CLASSIFY"
   - **Via Serial**: Type `1` + Enter in the serial monitor
   - Results are printed as `LABEL:<waste-type>` and forwarded to S3

---

## 🎯 How It Works

### Classification Pipeline

1. **Capture**: ESP32-CAM captures waste image
2. **Encode**: Image is JPEG-compressed and base64-encoded
3. **Send**: Image is transmitted to OpenRouter API with vision prompt
4. **Classify**: AI model identifies waste category
5. **Output**: Classification result (e.g., "plastic", "paper", "metal", "organic")
6. **Store**: Image and result are archived in AWS S3

### Supported Waste Categories

- ♻️ **Recyclables**: Plastic, paper, cardboard, glass, metal, aluminum
- 🥬 **Organic**: Food waste, compostables, green waste
- 🗑️ **General Waste**: Non-recyclable mixed materials
- ⚠️ **Special**: Hazardous materials, electronics, batteries

---

## 🔌 API Reference

### Web Endpoint

**GET** `/classify`
- Triggers a waste classification via button press on web UI
- Returns current classification result and status

**GET** `/` 
- Serves the web UI dashboard

### OpenRouter Integration

```c
POST https://openrouter.ai/api/v1/chat/completions
Headers:
  - Authorization: Bearer {OPENROUTER_API_KEY}
  - Content-Type: application/json

Body:
{
  "model": "openai/gpt-5.4",
  "messages": [
    {
      "role": "user",
      "content": [
        {
          "type": "image_url",
          "image_url": {"url": "data:image/jpeg;base64,..."}
        },
        {
          "type": "text",
          "text": "Classify this waste item into categories: plastic, paper, metal, glass, organic, or other."
        }
      ]
    }
  ]
}
```

---

## 📝 Configuration

### WiFi & Connectivity
```c
// camera_test/main/main.c
#define WIFI_STA_SSID       "Network Name"
#define WIFI_STA_PASS       "Password"
#define USE_PHONE_HOTSPOT   1  // Use phone hotspot for testing
```

### API Keys
```c
#define OPENROUTER_API_KEY  "sk-or-v1-..."
#define OPENROUTER_MODEL    "openai/gpt-5.4"
```

### AWS S3
```python
# bridge_to_s3.py
AWS_BUCKET_NAME = "your-bucket-name"
AWS_REGION = "us-east-1"
```

---

## 📊 Performance Metrics

- **Classification Latency**: ~2-5 seconds (includes capture, API call, and result)
- **Image Size**: ~50-100 KB (JPEG, optimized for streaming)
- **WiFi Requirement**: Minimum 2 Mbps for reliable operation
- **Accuracy**: Depends on OpenAI model and waste visibility

---

## 🔒 Security & Privacy

- ✅ API keys stored securely in configuration files (keep private)
- ✅ Images encrypted in transit to OpenRouter and AWS S3
- ✅ Optional: Implement local image deletion after S3 upload
- ⚠️ Recommendation: Use environment variables or secure key management for production

---

## 🤝 Contributing

We welcome contributions! To improve TrashTalker:

1. **Fork** the repository
2. **Create** a feature branch (`git checkout -b feature/amazing-feature`)
3. **Commit** changes (`git commit -m 'Add amazing feature'`)
4. **Push** to branch (`git push origin feature/amazing-feature`)
5. **Submit** a Pull Request

### Areas for Improvement
- [ ] Local ML model for offline classification
- [ ] Multiple waste type simultaneous detection
- [ ] Mobile app companion
- [ ] Database for classification history
- [ ] Community model improvement through user feedback

---

## 📄 License

This project is licensed under the **MIT License** – see [LICENSE](LICENSE) file for details.

---

## 🙏 Acknowledgments

- **Sodahacks 2025** – Hackathon organizers and judges
- **Espressif Systems** – ESP32 frameworks and libraries
- **OpenAI** – Vision models via OpenRouter API
- **AWS** – Cloud infrastructure and storage
- All contributors and supporters of sustainable waste management

---

## 📧 Contact & Support

For questions, issues, or suggestions:
- 🐛 **Report Issues**: [GitHub Issues](https://github.com/VamsiCC/Sodahacks_TrashTalker/issues)
- 💬 **Start Discussion**: [GitHub Discussions](https://github.com/VamsiCC/Sodahacks_TrashTalker/discussions)
- 🌐 **Website**: *(Add project website if available)*

---

<div align="center">
  <p><strong>♻️ Every classification counts toward a more sustainable future ♻️</strong></p>
  <p>Built with 💚 for Sodahacks 2025</p>
</div>
