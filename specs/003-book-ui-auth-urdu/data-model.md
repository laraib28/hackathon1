# Data Model: Book UI Enhancement, Better Auth Integration, and Urdu Translation Fix

**Feature**: 003-book-ui-auth-urdu
**Date**: 2025-12-18
**Status**: Design Complete

## Overview

This feature involves four main entities for authentication and chat functionality. The data model follows the existing backend implementation (SQLAlchemy + SQLite) with minimal changes.

## Entity Definitions

### 1. User

Represents an authenticated reader with account credentials and profile information.

**Attributes**:
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | String | PRIMARY KEY, UNIQUE | User identifier (generated token_urlsafe(16)) |
| email | String | UNIQUE, NOT NULL | User email address for login |
| password | String | NOT NULL | Hashed password (SHA-256, should be bcrypt in production) |
| name | String | NOT NULL | Display name |
| created_at | DateTime | DEFAULT NOW | Account creation timestamp |
| email_verified | Boolean | DEFAULT FALSE | Email verification status (not implemented yet) |

**Relationships**:
- One user → Many chat_history entries
- One user → Many user_sessions

**Validation Rules**:
- email: Must be valid email format (EmailStr in Pydantic)
- password: Minimum 6 characters (frontend validation)
- name: 1-100 characters

**State Transitions**:
```
[New] --signup--> [Active, Unverified] --email_verify--> [Active, Verified]
[Active] --delete--> [Deleted]
```

**Database Table** (SQLAlchemy):
```python
class User(Base):
    __tablename__ = "users"

    id = Column(String, primary_key=True)
    email = Column(String, unique=True, nullable=False, index=True)
    password = Column(String, nullable=False)  # Hashed
    name = Column(String, nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow)
    email_verified = Column(Boolean, default=False)

    # Relationships
    chat_history = relationship("ChatHistory", back_populates="user")
```

---

### 2. UserSession

Represents an active authentication session for a logged-in user.

**Attributes**:
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| token | String | PRIMARY KEY | Session token (secure random 32-byte URL-safe string) |
| user_id | String | FOREIGN KEY (users.id) | Reference to user |
| created_at | DateTime | DEFAULT NOW | Session creation timestamp |
| expires_at | DateTime | NOT NULL | Session expiration (7 days from creation) |

**Relationships**:
- One session → One user (many-to-one)

**Validation Rules**:
- token: 32-byte URL-safe random string
- expires_at: Must be > created_at
- Auto-cleanup: Expired sessions can be purged periodically

**State Transitions**:
```
[Created] --timeout--> [Expired]
[Created] --sign_out--> [Revoked]
```

**Current Implementation**:
```python
# In-memory sessions (backend/auth_api.py)
# Format: {token: {user_id, created_at, expires_at}}
active_sessions: Dict[str, Dict[str, Any]] = {}

# Production: Should be Redis or database table
```

**Future Database Table** (when migrating from in-memory):
```python
class UserSession(Base):
    __tablename__ = "user_sessions"

    token = Column(String, primary_key=True)
    user_id = Column(String, ForeignKey("users.id"), nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow)
    expires_at = Column(DateTime, nullable=False)
```

---

### 3. ChatMessage (Implicit)

Represents a user query sent to the chatbot. Not stored as separate entity currently; embedded in ChatHistory.

**Attributes** (from ChatRequest Pydantic model):
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| message | String | REQUIRED | User's chat message text |
| target_language | String | ENUM ('en', 'ur') | Detected language code |
| selected_text | String | OPTIONAL | Selected text from book for context |
| conversation_history | Array | OPTIONAL | Previous messages for context (last 6) |
| user_id | String | OPTIONAL | User ID if authenticated |

**Frontend Detection Logic**:
```typescript
// Detected via languageDetector.ts before sending to backend
target_language: detectLanguage(message) // 'en' | 'ur'
```

---

### 4. ChatHistory

Represents a logged chat interaction between user and chatbot.

**Attributes**:
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | Integer | PRIMARY KEY, AUTO_INCREMENT | Chat history entry ID |
| user_id | String | FOREIGN KEY (users.id) | User who sent the message |
| message | String | NOT NULL | User's original question |
| response | String | NOT NULL | Chatbot's answer |
| language | String | DEFAULT 'en' | Language code ('en' or 'ur') |
| sources | JSON | NULLABLE | RAG sources (array of {url, score}) |
| created_at | DateTime | DEFAULT NOW | Timestamp of interaction |

**Relationships**:
- One chat_history → One user (many-to-one)

**Validation Rules**:
- message: 1-1000 characters
- response: 1-5000 characters
- language: Must be 'en' or 'ur'
- sources: JSON array of objects with url (string) and score (float)

**Database Table** (SQLAlchemy):
```python
class ChatHistory(Base):
    __tablename__ = "chat_history"

    id = Column(Integer, primary_key=True, autoincrement=True)
    user_id = Column(String, ForeignKey("users.id"), nullable=True)  # Nullable for anonymous chat
    message = Column(String, nullable=False)
    response = Column(String, nullable=False)
    language = Column(String, default="en")
    sources = Column(JSON, nullable=True)
    created_at = Column(DateTime, default=datetime.utcnow)

    # Relationships
    user = relationship("User", back_populates="chat_history")
```

---

## Entity Relationships Diagram

```
User (1) ---> (*) UserSession
  |
  |
  v
  (*) ChatHistory
```

**Relationships**:
1. **User → UserSession**: One user can have multiple active sessions (e.g., desktop + mobile)
2. **User → ChatHistory**: One user can have many chat interactions logged

---

## Data Flow Diagrams

### Authentication Flow

```
[Client] --signup--> [POST /api/auth/sign-up/email]
    |
    v
[Create User] --hash password--> [Store in DB]
    |
    v
[Create Session] --generate token--> [Store in active_sessions]
    |
    v
[Return {user, session: {token}}] --store token--> [Client localStorage]
```

### Chat Flow

```
[Client] --detect language--> [languageDetector.ts]
    |
    v
[Send POST /api/chat {message, target_language}]
    |
    v
[Backend] --search--> [Qdrant Vector DB]
    |
    v
[Retrieve context] --prompt--> [OpenAI GPT-3.5]
    |
    v
[Generate response] --translate if ur--> [Return response]
    |
    v
[Log ChatHistory if user_id] --store--> [SQLite DB]
```

---

## Indexing Strategy

**Database Indexes** (for query performance):

```sql
-- User table
CREATE INDEX idx_users_email ON users(email);  -- Login lookup

-- ChatHistory table
CREATE INDEX idx_chat_history_user_id ON chat_history(user_id);  -- User's chat history
CREATE INDEX idx_chat_history_created_at ON chat_history(created_at DESC);  -- Recent chats
```

**Notes**:
- UserSession doesn't need additional indexes (in-memory dict, O(1) lookup)
- When migrating to database sessions, add index on user_id and expires_at

---

## Data Migration Considerations

### Current State → Target State

**No breaking changes required**:
- User table: Already exists in database.py
- ChatHistory table: Already exists in database.py
- UserSession: Currently in-memory, migration to DB is future enhancement (out of scope)

**Legacy cleanup**:
- Remove Firebase user records (if any exist)
- No data migration needed (fresh feature deployment)

---

## Security Considerations

**Password Security**:
- ⚠️ Current: SHA-256 hashing (backend/auth_api.py line 54-56)
- ✅ Recommended: Upgrade to bcrypt with salt (future task)

**Session Security**:
- ✅ Token generation: `secrets.token_urlsafe(32)` (cryptographically secure)
- ⚠️ Session storage: In-memory dict (production should use Redis)
- ✅ Token transmission: HTTPS only, Bearer header
- ❌ Missing: HttpOnly cookies (tokens stored in localStorage, vulnerable to XSS)

**Data Privacy**:
- Chat history: Stored with user_id, consider retention policy
- No PII beyond email/name in User table
- Sources field may contain book URLs (public data)

---

## Summary

| Entity | Storage | Primary Key | Relationships | State |
|--------|---------|-------------|---------------|-------|
| User | SQLite (database.py) | id (String) | → UserSession, → ChatHistory | ✅ Implemented |
| UserSession | In-memory dict | token (String) | → User | ⚠️ In-memory (migrate to DB later) |
| ChatHistory | SQLite (database.py) | id (Integer) | → User | ✅ Implemented |

**Data Model Status**: ✅ Complete - No schema changes needed for this feature

**Next Step**: Define API contracts for authentication and chat endpoints.
