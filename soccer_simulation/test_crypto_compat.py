import sys
print(f"Python Version: {sys.version}")

try:
    from cryptography.hazmat.primitives import hashes
    from cryptography.hazmat.primitives.asymmetric import padding
    from cryptography.hazmat.primitives.asymmetric import rsa
    from cryptography.exceptions import InvalidSignature
    
    print("Cryptography module imported successfully.")
    
    # 1. Generate Key
    private_key = rsa.generate_private_key(
        public_exponent=65537,
        key_size=2048,
    )
    public_key = private_key.public_key()
    print("RSA Key Generated.")
    
    # 2. Sign
    message = b"A message for signing"
    signature = private_key.sign(
        message,
        padding.PKCS1v15(),
        hashes.SHA256()
    )
    print(f"Available Signature generated. Len: {len(signature)}")
    
    # 3. Verify
    try:
        public_key.verify(
            signature,
            message,
            padding.PKCS1v15(),
            hashes.SHA256()
        )
        print("Signature Verified Successfully. Crypto math is working.")
    except InvalidSignature:
        print("FAIL: Signature Verification Failed! Library is broken.")
        
except Exception as e:
    print(f"CRITICAL ERROR: {e}")
