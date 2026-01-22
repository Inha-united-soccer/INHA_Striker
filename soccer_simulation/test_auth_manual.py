import json
import time
import requests
import base64
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.asymmetric import padding
from cryptography.hazmat.primitives import serialization

KEY_PATH = "serviceAccountKey.json"

def b64_enc(data):
    return base64.urlsafe_b64encode(data).rstrip(b'=')

def test_manual_jwt():
    print("=== MANUAL JWT AUTH TEST ===")
    
    # 1. Load Key
    with open(KEY_PATH, "r") as f:
        data = json.load(f)
    
    private_key_pem = data["private_key"].strip().replace("\\n", "\n").encode('utf-8')
    client_email = data["client_email"]
    
    # 2. Load Private Key Object
    try:
        private_key = serialization.load_pem_private_key(
            private_key_pem,
            password=None
        )
        print("[OK] Private Key Loaded via Cryptography")
    except Exception as e:
        print(f"[FAIL] Key Load Error: {e}")
        return

    # 3. Construct JWT
    now = int(time.time())
    header = {
        "alg": "RS256",
        "typ": "JWT"
    }
    
    payload = {
        "iss": client_email,
        "sub": client_email,
        "aud": "https://oauth2.googleapis.com/token",
        "iat": now,
        "exp": now + 3600,
        "scope": "https://www.googleapis.com/auth/firebase.database https://www.googleapis.com/auth/userinfo.email"
    }
    
    h_bytes = b64_enc(json.dumps(header).encode('utf-8'))
    p_bytes = b64_enc(json.dumps(payload).encode('utf-8'))
    to_sign = h_bytes + b'.' + p_bytes
    
    signature = private_key.sign(
        to_sign,
        padding.PKCS1v15(),
        hashes.SHA256()
    )
    s_bytes = b64_enc(signature)
    
    jwt_token = (h_bytes + b'.' + p_bytes + b'.' + s_bytes).decode('utf-8')
    
    print(f"\n[Generated JWT - First 50 chars]: {jwt_token[:50]}...")
    print(f"Time used: {now}")

    # 4. Exchange for Access Token
    url = "https://oauth2.googleapis.com/token"
    params = {
        "grant_type": "urn:ietf:params:oauth:grant-type:jwt-bearer",
        "assertion": jwt_token
    }
    
    print(f"\nSending Request to {url}...")
    resp = requests.post(url, data=params)
    
    print(f"Status Code: {resp.status_code}")
    print(f"Response: {resp.text}")

if __name__ == "__main__":
    test_manual_jwt()
