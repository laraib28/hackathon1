"""
Better Auth Compatible API for FastAPI
Provides authentication endpoints compatible with Better Auth React client
"""

from fastapi import APIRouter, HTTPException, Depends, Header
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from pydantic import BaseModel, EmailStr
from typing import Optional, Dict, Any
import secrets
import hashlib
from datetime import datetime, timedelta
from sqlalchemy.orm import Session

# Import database functions
from database import (
    get_db,
    create_user,
    get_user_by_email,
    get_user_by_id,
)

# Create router
router = APIRouter()

# Security scheme
security = HTTPBearer(auto_error=False)

# In-memory sessions (in production, use Redis or database)
# Format: {token: {user_id, created_at, expires_at}}
active_sessions: Dict[str, Dict[str, Any]] = {}

# -------------------------------------------------------
# MODELS
# -------------------------------------------------------

class SignUpEmailRequest(BaseModel):
    email: EmailStr
    password: str
    name: Optional[str] = None

class SignInEmailRequest(BaseModel):
    email: EmailStr
    password: str

class SessionResponse(BaseModel):
    user: Optional[Dict[str, Any]] = None
    session: Optional[Dict[str, Any]] = None

# -------------------------------------------------------
# HELPER FUNCTIONS
# -------------------------------------------------------

def hash_password(password: str) -> str:
    """Simple password hashing (use bcrypt in production)"""
    return hashlib.sha256(password.encode()).hexdigest()

def generate_session_token() -> str:
    """Generate secure session token"""
    return secrets.token_urlsafe(32)

def create_session(user_id: str) -> str:
    """Create new session and return token"""
    token = generate_session_token()
    active_sessions[token] = {
        "user_id": user_id,
        "created_at": datetime.utcnow(),
        "expires_at": datetime.utcnow() + timedelta(days=7)
    }
    return token

def verify_session(token: str) -> Optional[str]:
    """Verify session token and return user_id"""
    if token not in active_sessions:
        return None

    session = active_sessions[token]
    if datetime.utcnow() > session["expires_at"]:
        del active_sessions[token]
        return None

    return session["user_id"]

def get_current_user_from_header(authorization: Optional[str] = Header(None), db: Session = Depends(get_db)) -> Optional[Dict]:
    """Extract user from Authorization header"""
    if not authorization:
        return None

    # Handle Bearer token
    if authorization.startswith("Bearer "):
        token = authorization.replace("Bearer ", "")
        user_id = verify_session(token)

        if user_id:
            user = get_user_by_id(db, user_id)
            if user:
                return {
                    "id": user.id,
                    "email": user.email,
                    "name": user.name,
                    "emailVerified": False,
                    "createdAt": user.created_at.isoformat() if hasattr(user, 'created_at') else None
                }

    return None

# -------------------------------------------------------
# AUTH ENDPOINTS (Better Auth Compatible)
# -------------------------------------------------------

@router.post("/sign-up/email")
async def sign_up_email(request: SignUpEmailRequest, db: Session = Depends(get_db)):
    """
    Better Auth compatible email signup endpoint
    POST /api/auth/sign-up/email
    """
    # Check if email already exists
    existing_user = get_user_by_email(db, request.email)
    if existing_user:
        raise HTTPException(status_code=400, detail="Email already registered")

    # Generate user ID
    user_id = secrets.token_urlsafe(16)

    # Create user
    user_data = {
        "id": user_id,
        "email": request.email,
        "password": hash_password(request.password),
        "name": request.name or request.email.split('@')[0]
    }

    user = create_user(db, user_data)

    # Create session
    token = create_session(user.id)

    return {
        "user": {
            "id": user.id,
            "email": user.email,
            "name": user.name,
            "emailVerified": False,
            "createdAt": datetime.utcnow().isoformat()
        },
        "session": {
            "token": token,
            "expiresAt": (datetime.utcnow() + timedelta(days=7)).isoformat()
        }
    }

@router.post("/sign-in/email")
async def sign_in_email(request: SignInEmailRequest, db: Session = Depends(get_db)):
    """
    Better Auth compatible email sign-in endpoint
    POST /api/auth/sign-in/email
    """
    # Find user
    user = get_user_by_email(db, request.email)

    if not user:
        raise HTTPException(status_code=401, detail="Invalid credentials")

    # Verify password
    if not hasattr(user, 'password') or user.password != hash_password(request.password):
        raise HTTPException(status_code=401, detail="Invalid credentials")

    # Create session
    token = create_session(user.id)

    return {
        "user": {
            "id": user.id,
            "email": user.email,
            "name": user.name,
            "emailVerified": False,
            "createdAt": getattr(user, 'created_at', datetime.utcnow()).isoformat() if hasattr(user, 'created_at') else datetime.utcnow().isoformat()
        },
        "session": {
            "token": token,
            "expiresAt": (datetime.utcnow() + timedelta(days=7)).isoformat()
        }
    }

@router.post("/sign-out")
async def sign_out(credentials: HTTPAuthorizationCredentials = Depends(security)):
    """
    Better Auth compatible sign-out endpoint
    POST /api/auth/sign-out
    """
    if credentials:
        token = credentials.credentials
        if token in active_sessions:
            del active_sessions[token]

    return {"success": True}

@router.get("/session")
async def get_session(authorization: Optional[str] = Header(None), db: Session = Depends(get_db)) -> SessionResponse:
    """
    Better Auth compatible session endpoint
    GET /api/auth/session
    Returns current user session
    """
    user_data = get_current_user_from_header(authorization, db)

    if user_data:
        # Extract token from header
        token = authorization.replace("Bearer ", "") if authorization and authorization.startswith("Bearer ") else None

        return SessionResponse(
            user=user_data,
            session={"token": token} if token else None
        )

    return SessionResponse(user=None, session=None)

# -------------------------------------------------------
# DEPENDENCY FOR PROTECTED ROUTES
# -------------------------------------------------------

async def require_auth(authorization: Optional[str] = Header(None), db: Session = Depends(get_db)) -> Dict:
    """
    Dependency that requires authentication
    Use this in protected routes: user = Depends(require_auth)
    """
    user = get_current_user_from_header(authorization, db)

    if not user:
        raise HTTPException(
            status_code=401,
            detail="Authentication required. Please log in."
        )

    return user

# Export router
__all__ = ["router", "require_auth"]
