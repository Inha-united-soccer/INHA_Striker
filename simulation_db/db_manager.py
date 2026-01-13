import sqlite3
import json
import time
import os

# Firebase Imports
try:
    import firebase_admin
    from firebase_admin import credentials
    from firebase_admin import db
    FIREBASE_AVAILABLE = True
except ImportError:
    FIREBASE_AVAILABLE = False
    print("[DB] firebase-admin not installed. Cloud sync disabled.")

DB_NAME = "soccer_sim.db"
KEY_PATH = "serviceAccountKey.json"
# IMPORTANT: Set this to your Firebase Database URL
FIREBASE_DB_URL = "https://soccer-db-f6361-default-rtdb.firebaseio.com/"

class DBManager:
    def __init__(self, db_name=DB_NAME):
        self.db_name = db_name
        self.firebase_app = None
        self.init_db()
        self.init_firebase()

    def init_db(self):
        conn = sqlite3.connect(self.db_name)
        cursor = conn.cursor()
        
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS sim_runs (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp REAL,
                duration REAL,
                total_striker_score REAL,
                pass_success_rate REAL,
                note TEXT
            )
        ''')

        cursor.execute('''
            CREATE TABLE IF NOT EXISTS sim_params (
                run_id INTEGER,
                param_type TEXT,
                param_key TEXT,
                param_value REAL,
                FOREIGN KEY(run_id) REFERENCES sim_runs(id)
            )
        ''')
        
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS sim_logs (
                run_id INTEGER,
                log_json TEXT,
                FOREIGN KEY(run_id) REFERENCES sim_runs(id)
            )
        ''')
        
        conn.commit()
        conn.close()

    def init_firebase(self):
        if not FIREBASE_AVAILABLE: return
        
        # Avoid re-initialization on Streamlit reload
        if firebase_admin._apps:
            self.firebase_app = firebase_admin.get_app()
            return

        # 1. Try Streamlit Secrets (Cloud)
        # 1. Try Streamlit Secrets (Cloud)
        try:
            import streamlit as st
            if hasattr(st, "secrets"):
                # Debug: Print available secret keys (safely)
                print(f"[DB] Available Secret Keys: {list(st.secrets.keys())}")
                
                key_dict = None
                
                # Case A: [firebase] section exists
                if "firebase" in st.secrets:
                    print("[DB] Found [firebase] section in secrets.")
                    key_dict = dict(st.secrets["firebase"])
                
                # Case B: Keys are at root level (User pasted JSON directly)
                elif "private_key" in st.secrets and "project_id" in st.secrets:
                    print("[DB] Found secrets at root level.")
                    key_dict = dict(st.secrets)
                
                if key_dict:
                    try:
                        # Fix private_key formatting
                        if "private_key" in key_dict:
                            key_dict["private_key"] = key_dict["private_key"].replace("\\n", "\n")

                        cred = credentials.Certificate(key_dict)
                        self.firebase_app = firebase_admin.initialize_app(cred, {
                            'databaseURL': FIREBASE_DB_URL
                        })
                        print("[DB] Firebase Connected via Secrets.")
                        return
                    except Exception as e:
                        print(f"[DB] Secrets Init Failed: {e}")
        except ImportError:
            pass

        # 2. Try Local File
        if os.path.exists(KEY_PATH):
            try:
                cred = credentials.Certificate(KEY_PATH)
                self.firebase_app = firebase_admin.initialize_app(cred, {
                    'databaseURL': FIREBASE_DB_URL
                })
                print("[DB] Firebase Connected Successfully.")
            except Exception as e:
                print(f"[DB] Firebase Init Failed: {e}")
        else:
            print(f"[DB] {KEY_PATH} not found. Local only mode.")

    def save_run(self, duration, log_data, config_pass, config_st):
        """
        Saves a completed simulation run to Local SQLite AND Firebase (if connected).
        """
        # 1. Local Save
        conn = sqlite3.connect(self.db_name)
        cursor = conn.cursor()
        
        timestamp = time.time()
        avg_score = 0.0
        if len(log_data) > 0:
            avg_score = sum(float(row['ofb_score']) for row in log_data) / len(log_data)
            
        cursor.execute('''
            INSERT INTO sim_runs (timestamp, duration, total_striker_score, pass_success_rate, note)
            VALUES (?, ?, ?, ?, ?)
        ''', (timestamp, duration, avg_score, 0.0, "Auto-saved run"))
        
        run_id = cursor.lastrowid
        
        for k, v in config_pass.items():
            cursor.execute('INSERT INTO sim_params VALUES (?, ?, ?, ?)', (run_id, 'PASS', k, float(v)))
        for k, v in config_st.items():
            cursor.execute('INSERT INTO sim_params VALUES (?, ?, ?, ?)', (run_id, 'ST', k, float(v)))
            
        log_json = json.dumps(log_data)
        cursor.execute('INSERT INTO sim_logs VALUES (?, ?)', (run_id, log_json))
        
        conn.commit()
        conn.close()
        print(f"[DB] Local Run saved with ID: {run_id}")
        
        # 2. Cloud Save
        self.upload_to_firebase(run_id, timestamp, duration, avg_score, config_pass, config_st, log_data)

        return run_id

    def upload_to_firebase(self, local_id, timestamp, duration, score, cfg_pass, cfg_st, logs):
        if not self.firebase_app: return
        
        try:
            root = db.reference('sim_runs')
            new_run_ref = root.push() # Create unique ID
            
            data = {
                "local_id": local_id,
                "timestamp": timestamp,
                "duration": duration,
                "score": score,
                "config": {
                    "PASS": cfg_pass,
                    "ST": cfg_st
                },
                "logs_count": len(logs),
                # "logs": logs # Uncomment to upload full logs (heavy bandwidth!)
            }
            
            new_run_ref.set(data)
            print(f"[DB] Uploaded to Firebase: {new_run_ref.key}")
            
        except Exception as e:
            print(f"[DB] Upload Failed: {e}")

    def get_runs(self, limit=10):
        # Local view only
        conn = sqlite3.connect(self.db_name)
        cursor = conn.cursor()
        cursor.execute('SELECT * FROM sim_runs ORDER BY id DESC LIMIT ?', (limit,))
        rows = cursor.fetchall()
        conn.close()
        return rows
