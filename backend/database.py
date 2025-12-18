"""
Neon Postgres Database Setup
Manages user profiles, preferences, and personalization data
"""

from sqlalchemy import create_engine, Column, String, Integer, DateTime, Text, JSON
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from datetime import datetime
import os
from dotenv import load_dotenv

load_dotenv()

# Neon Postgres connection
DATABASE_URL = "postgresql://neondb_owner:npg_w1f2YebzEmSv@ep-wispy-salad-a1cfpxoq-pooler.ap-southeast-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require"

engine = create_engine(DATABASE_URL)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base = declarative_base()

class User(Base):
    """User profile with background information"""
    __tablename__ = "users"

    id = Column(String, primary_key=True)
    email = Column(String, unique=True, nullable=False)
    password = Column(String, nullable=False)  # Password hash
    name = Column(String, nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

    # Background information for personalization
    software_experience = Column(String)  # beginner, intermediate, advanced, expert
    hardware_experience = Column(String)  # none, basic, intermediate, advanced
    programming_level = Column(String)  # beginner, intermediate, advanced
    programming_languages = Column(JSON)  # List of languages
    learning_goals = Column(Text)
    industry_background = Column(String)

    # Preferences
    preferred_language = Column(String, default="en")
    content_difficulty = Column(String, default="intermediate")

class ContentPreference(Base):
    """User's content personalization preferences"""
    __tablename__ = "content_preferences"

    id = Column(Integer, primary_key=True, autoincrement=True)
    user_id = Column(String, nullable=False)
    chapter_id = Column(String, nullable=False)
    personalized = Column(Integer, default=0)  # Boolean: 0 or 1
    translated = Column(Integer, default=0)  # Boolean: 0 or 1
    target_language = Column(String, default="en")
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

class ChatHistory(Base):
    """Store chat interactions for context"""
    __tablename__ = "chat_history"

    id = Column(Integer, primary_key=True, autoincrement=True)
    user_id = Column(String, nullable=False)
    message = Column(Text, nullable=False)
    response = Column(Text, nullable=False)
    language = Column(String, default="en")
    sources = Column(JSON)  # Search results used
    created_at = Column(DateTime, default=datetime.utcnow)

def init_db():
    """Initialize database tables"""
    Base.metadata.create_all(bind=engine)
    print("✅ Database tables created successfully")

def get_db():
    """Get database session"""
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

def create_user(db, user_data: dict):
    """Create new user profile"""
    user = User(**user_data)
    db.add(user)
    db.commit()
    db.refresh(user)
    return user

def get_user_by_email(db, email: str):
    """Get user by email"""
    return db.query(User).filter(User.email == email).first()

def get_user_by_id(db, user_id: str):
    """Get user by ID"""
    return db.query(User).filter(User.id == user_id).first()

def update_user_background(db, user_id: str, background_data: dict):
    """Update user background information"""
    user = get_user_by_id(db, user_id)
    if user:
        for key, value in background_data.items():
            if hasattr(user, key):
                setattr(user, key, value)
        db.commit()
        db.refresh(user)
    return user

def save_chat_history(db, user_id: str, message: str, response: str, language: str, sources: list):
    """Save chat interaction"""
    chat = ChatHistory(
        user_id=user_id,
        message=message,
        response=response,
        language=language,
        sources=sources
    )
    db.add(chat)
    db.commit()
    return chat

def get_content_preference(db, user_id: str, chapter_id: str):
    """Get user's preference for a specific chapter"""
    return db.query(ContentPreference).filter(
        ContentPreference.user_id == user_id,
        ContentPreference.chapter_id == chapter_id
    ).first()

def save_content_preference(db, user_id: str, chapter_id: str, personalized: bool, translated: bool, target_language: str = "en"):
    """Save or update content preference"""
    pref = get_content_preference(db, user_id, chapter_id)

    if pref:
        pref.personalized = 1 if personalized else 0
        pref.translated = 1 if translated else 0
        pref.target_language = target_language
        pref.updated_at = datetime.utcnow()
    else:
        pref = ContentPreference(
            user_id=user_id,
            chapter_id=chapter_id,
            personalized=1 if personalized else 0,
            translated=1 if translated else 0,
            target_language=target_language
        )
        db.add(pref)

    db.commit()
    db.refresh(pref)
    return pref

if __name__ == "__main__":
    print("=" * 60)
    print("🗄️  Initializing Neon Postgres Database")
    print("=" * 60)
    init_db()
    print("\n✅ Database setup complete!")
