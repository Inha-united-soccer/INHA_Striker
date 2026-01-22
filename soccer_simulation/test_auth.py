import os
import json
import time
import requests
import email.utils
import firebase_admin
from firebase_admin import credentials, db

KEY_PATH = "serviceAccountKey.json"
DATABASE_URL = "https://soccer-db-f6361-default-rtdb.firebaseio.com/"

def test_auth():
    print("=== FIREBASE AUTH DIAGNOSTIC TOOL ===")
    
    # 1. Time Check
    print("\n[1] Checking System Time vs Google Server Time...")
    try:
        resp = requests.head("https://www.google.com", timeout=2)
        server_date = resp.headers['date']
        server_ts = email.utils.mktime_tz(email.utils.parsedate_tz(server_date))
        local_ts = time.time()
        drift = server_ts - local_ts
        print(f"    System Time: {local_ts}")
        print(f"    Server Time: {server_ts}")
        print(f"    Drift: {drift:.2f} seconds")
        
        if abs(drift) > 300:
            print("    [!] WARNING: Large Time Skew detected. This causes JWT errors.")
            print("    Applying Monkey Patch...")
            _orig_time = time.time
            time.time = lambda: _orig_time() + drift
        else:
            print("    [OK] Time sync looks good.")
            
    except Exception as e:
        print(f"    [!] Checking time failed: {e}")

    # 2. Key File Check
    print("\n[2] Checking serviceAccountKey.json...")
    if not os.path.exists(KEY_PATH):
        print(f"    [ERROR] File {KEY_PATH} NOT FOUND in {os.getcwd()}")
        return

    try:
        with open(KEY_PATH, "r") as f:
            data = json.load(f)
        
        pk = data.get("private_key", "")
        pid = data.get("private_key_id", "UNKNOWN")
        client_email = data.get("client_email", "UNKNOWN")
        
        print(f"    Project ID: {data.get('project_id')}")
        print(f"    Client Email: {client_email}")
        print(f"    Private Key ID: {pid}")
        print(f"    Private Key Length: {len(pk)}")
        
        if "-----BEGIN PRIVATE KEY-----" not in pk:
            print("    [ERROR] Private Key Header missing!")
        else:
            print("    [OK] Key Header found.")
            
        # Clean Key Logic
        data["private_key"] = pk.strip().replace("\\n", "\n")
        
        cred = credentials.Certificate(data)
        print("    [OK] Credential Object created successfully.")

        # 2.5 Low-Level Signer Check
        print("\n[2.5] Testing Crypto Signer...")
        from google.auth import crypt
        try:
             signer = crypt.RSASigner.from_string(data["private_key"], data["private_key_id"])
             print(f"    [OK] RSASigner loaded key successfully. Key ID: {signer.key_id}")
             
             # Test Sign
             sig = signer.sign(b"test")
             print(f"    [OK] Signed dummy data. Sig Len: {len(sig)}")
        except Exception as e:
             print(f"    [ERROR] RSASigner Failed: {e}")
        
    except Exception as e:
        print(f"    [ERROR] Failed to load/parse JSON: {e}")
        return

    # 3. Connection Test
    print("\n[3] Testing Firebase Connection...")
    try:
        if not firebase_admin._apps:
            firebase_admin.initialize_app(cred, {'databaseURL': DATABASE_URL})
        
        ref = db.reference('test_connection')
        ref.set({"status": "ok", "timestamp": time.time()})
        print("    [SUCCESS] Successfully wrote to database!")
        print("    You are ready to go.")
        
    except Exception as e:
        print(f"    [FAIL] Connection Failed: {e}")
        print("    Possible causes:")
        print("    - Service Account revoked")
        print("    - Database URL incorrect")
        print("    - System Time mismatch (if not patched)")

if __name__ == "__main__":
    test_auth()
