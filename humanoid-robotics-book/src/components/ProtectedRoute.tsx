/**
 * ProtectedRoute Component
 * Wraps protected content and redirects unauthenticated users to login
 */
import React from 'react';
import { useLocation, useNavigate } from '@docusaurus/router';
import { useAuth } from '../context/AuthContext';

interface ProtectedRouteProps {
  children: React.ReactNode;
  publicPages?: string[]; // Pages that are accessible without authentication
}

const ProtectedRoute: React.FC<ProtectedRouteProps> = ({
  children,
  publicPages = ['/', '/login', '/signup']
}) => {
  const { currentUser, loading } = useAuth();
  const location = useLocation();
  const navigate = useNavigate();

  // Check if current path is a public page, including i18n versions
  const isPublicPage = publicPages.some(publicPath => {
    // Exact match
    if (location.pathname === publicPath) return true;
    // Match with locale prefix (e.g., /ur/login matches /login pattern)
    if (location.pathname === `/ur${publicPath}`) return true;
    // Match root locale pages
    if (publicPath === '/' && (location.pathname === '/ur' || location.pathname === '/ur/')) return true;
    return false;
  });

  // Define routes that should be protected (all /docs/* routes and chapter routes, including i18n versions)
  const isProtectedRoute = (/^\/(part1-foundations|part2-modules|part3-capstone|part4-future|docs)/.test(location.pathname) ||
                           /^\/ur\/(part1-foundations|part2-modules|part3-capstone|part4-future|docs)/.test(location.pathname))
    && !isPublicPage;

  // If loading, show a loading state
  if (loading) {
    return (
      <div style={{
        display: 'flex',
        justifyContent: 'center',
        alignItems: 'center',
        height: '100vh',
        fontSize: '18px',
        color: '#555'
      }}>
        Loading...
      </div>
    );
  }

  // Allow access if user is authenticated OR it's a public page
  if (currentUser || !isProtectedRoute) {
    return <>{children}</>;
  }

  // Redirect to login for protected content
  // Store the attempted route for redirect after login
  const from = location.pathname + location.search;
  navigate(`/login?redirect=${encodeURIComponent(from)}`, { replace: true });

  return null;
};

export default ProtectedRoute;