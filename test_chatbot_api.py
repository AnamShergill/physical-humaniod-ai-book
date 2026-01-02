import requests
import json

# Test the chatbot API - query parameter goes in the URL
base_url = "http://localhost:8000/api/v1/chatbot/query"

# Test with query parameter in the URL
params = {
    "query": "What is Physical AI?"
}

# Also try a more general query that should match textbook content
params2 = {
    "query": "What are humanoid robots?"
}

# Empty JSON body since query is in URL params
data = {}

try:
    # Test first query
    print("Testing first query: What is Physical AI?")
    response = requests.post(base_url, params=params, json=data)
    print(f"Status Code: {response.status_code}")
    print(f"Response: {response.text}")

    if response.status_code == 200:
        response_json = response.json()
        print("\nFormatted Response:")
        print(json.dumps(response_json, indent=2))
    else:
        print(f"\nError: {response.status_code}")

    print("\n" + "="*50 + "\n")

    # Test second query
    print("Testing second query: What are humanoid robots?")
    response2 = requests.post(base_url, params=params2, json=data)
    print(f"Status Code: {response2.status_code}")
    print(f"Response: {response2.text}")

    if response2.status_code == 200:
        response_json2 = response2.json()
        print("\nFormatted Response:")
        print(json.dumps(response_json2, indent=2))
    else:
        print(f"\nError: {response2.status_code}")

except requests.exceptions.ConnectionError:
    print("Could not connect to the API. Is the server running on localhost:8000?")
except Exception as e:
    print(f"An error occurred: {e}")