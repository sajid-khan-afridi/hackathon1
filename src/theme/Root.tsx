/**
 * Root component wrapper for Docusaurus
 *
 * This component wraps the entire Docusaurus site and allows us to
 * add global components like the ChatInterface floating widget.
 *
 * The ChatInterface will be available on all pages of the site.
 */

import React from 'react';
import ChatInterface from '@site/src/components/ChatInterface/ChatInterface';

// Props type matching Docusaurus Root component
interface RootProps {
  children: React.ReactNode;
}

export default function Root({ children }: RootProps): React.JSX.Element {
  return (
    <>
      {children}
      <ChatInterface />
    </>
  );
}
