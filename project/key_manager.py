import os
from cryptography.fernet import Fernet

# Get the absolute path to the key file
KEY_FILE_PATH = os.path.join(os.path.dirname(__file__), "encryption.key")

def load_key():
    """Load the encryption key from a file."""
    try:
        with open(KEY_FILE_PATH, "rb") as key_file:
            return key_file.read()
    except FileNotFoundError:
        print(f"❌ Error: Encryption key not found at {KEY_FILE_PATH}")
        raise  # Re-raise the exception

def generate_key():
    """Generate a new encryption key and save it to a file."""
    key = Fernet.generate_key()
    with open(KEY_FILE_PATH, "wb") as key_file:
        key_file.write(key)
    print(f"🔑 New encryption key generated at {KEY_FILE_PATH}")

# Load or generate key if it doesn't exist
if not os.path.exists(KEY_FILE_PATH):
    generate_key()

encryption_key = load_key()
