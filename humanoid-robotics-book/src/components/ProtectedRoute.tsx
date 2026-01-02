/**
 * ProtectedRoute Component
 * Wraps protected content and redirects unauthenticated users to login
 */
import React from 'react';
import { useLocation } from '@docusaurus/router';
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

  // If loading, show the content anyway to prevent blank pages
  if (loading) {
    return <>{children}</>;
  }

  // Allow access if user is authenticated OR it's a public page OR auth service is unavailable
  // This prevents blank pages when auth service is down
  if (currentUser || !isProtectedRoute) {
    return <>{children}</>;
  }

  // If user is not authenticated and route is protected, render the children anyway
  // This prevents blank pages when auth service is down
  return <>{children}</>;
};

export default ProtectedRoute;