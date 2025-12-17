/**
 * ProtectedRoute Component
 * Redirects unauthenticated users to login page
 * Preserves the intended destination URL for redirect after login
 */
import React, { useEffect } from 'react';
import { useAuth } from '../../context/AuthContext';
import BrowserOnly from '@docusaurus/BrowserOnly';

interface ProtectedRouteProps {
  children: React.ReactNode;
}

function ProtectedRouteContent({ children }: ProtectedRouteProps) {
  const { currentUser, loading } = useAuth();

  useEffect(() => {
    // Only redirect if loading is complete and no user is found
    if (!loading && !currentUser) {
      // Save current path for redirect after login
      const currentPath = window.location.pathname + window.location.search;
      const loginUrl = `/login?redirect=${encodeURIComponent(currentPath)}`;

      // Redirect to login
      window.location.href = loginUrl;
    }
  }, [currentUser, loading]);

  // Show loading spinner while checking authentication
  if (loading) {
    return (
      <div style={{
        display: 'flex',
        justifyContent: 'center',
        alignItems: 'center',
        minHeight: '400px'
      }}>
        <div style={{
          fontSize: '18px',
          color: '#666'
        }}>
          Loading...
        </div>
      </div>
    );
  }

  // If not authenticated, show nothing (will redirect)
  if (!currentUser) {
    return null;
  }

  // User is authenticated, render children
  return <>{children}</>;
}

export default function ProtectedRoute({ children }: ProtectedRouteProps) {
  // Wrap in BrowserOnly to ensure this only runs on client-side
  return (
    <BrowserOnly fallback={<div>Loading...</div>}>
      {() => <ProtectedRouteContent>{children}</ProtectedRouteContent>}
    </BrowserOnly>
  );
}
